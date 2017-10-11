#!/usr/bin/env python2

import rospy
from nmea_msgs.msg import Sentence

from serial import Serial, SerialException

from threading import Thread, Lock
from httplib import HTTPConnection, HTTPException
from base64 import b64encode


class NTRIPClient:
    def __init__(self, ntrip_configs, serial_configs):
        # GGA
        self.serial_configs = serial_configs
        self.gga_lock = Lock()
        self.latest_gga = ''
        self.has_new_gga = False
        self.leftover_gga = ''
        self.nmea_pub = rospy.Publisher('nmea_sentence',
                                        Sentence,
                                        queue_size=100)
        self.seq = 0
        self.serial_lock = Lock()
        try:
            self.serial = Serial(serial_configs['port'],
                                 serial_configs['baud'],
                                 timeout=5.0)
        except SerialException as e:
            rospy.logerr("Error opening serial: %s", e)
            rospy.signal_shutdown("Error opening serial: %s" % e)
            raise SystemExit

        # RTCM
        self.ntrip_configs = ntrip_configs
        user_pass = '{0}:{1}'.format(ntrip_configs['user'],
                                     ntrip_configs['password'])
        self.header = {
            'Ntrip-Version': 'Ntrip/2.0',
            'User-Agent': 'NTRIP ntrip_client',
            'Connection': 'close',
            'Authorization': 'Basic ' + b64encode(user_pass)
        }
        self.connection = HTTPConnection(ntrip_configs['server'],
                                         int(ntrip_configs['port']))
        self.rtcm_thread = Thread(target=self.rtcm_thread_func,
                                  name='rtcm_thread')
        self.rtcm_thread.start()

    def publish_gga_sentence(self):
        # Read GGA sentence
        msg = Sentence()
        msg.header.frame_id = 'gps_origin'
        msg.header.seq = self.seq
        self.seq += 1
        msg.header.stamp = rospy.Time.now()
        msg.sentence = self.read_gga_sentence()
        if not NTRIPClient.is_good_gga(msg.sentence):
            return
        self.nmea_pub.publish(msg)

    def read_gga_sentence(self):
        data = ''
        with self.serial_lock:
            try:
                data = self.serial.readline()
            except SerialException:
                rospy.logwarn('Serial port read failed')
        with self.gga_lock:
            self.latest_gga = data
            self.has_new_gga = True
        return data

    def rtcm_thread_func(self):
        # Open connection
        while not rospy.is_shutdown():
            try:
                self.connection.request('GET', '/' + self.ntrip_configs['stream'])
                response = self.connection.getresponse()
                if response.status == 200:
                    break
            except HTTPException:
                rospy.logerr("Couldn't send request")

        # Get RTCM data
        while not rospy.is_shutdown():
            # Get latest GGA
            with self.gga_lock:
                gga = self.latest_gga
                self.has_new_gga = False
            if not NTRIPClient.is_good_gga(gga, bad_data_quality=['0']):
                continue

            # Send to NTRIP server
            try:
                self.connection.putheader(self.header)
                self.connection.endheaders()
                self.connection.send(gga)
            except HTTPException:
                rospy.logerr("Couldn't send GGA sentence")
                continue

            # Parse response
            try:
                response = self.connection.getresponse()
            except HTTPException:
                rospy.logerr("Couldn't get response")
                continue
            if response.status != 200:
                err = 'NTRIP Server response: {0}'.format(response.status)
                rospy.logerr(err)
                continue
            self.read_rtcm(response)

    def read_rtcm(self, response):
        # Keep reading RTCM and send it to the receiver
        while True:
            with self.gga_lock:
                # Return if there's newer GGA
                if rospy.is_shutdown() or self.has_new_gga:
                    return

            rtcm = response.read()
            # Write to GNSS receiver
            with self.serial_lock:
                self.serial.write(rtcm)

    @staticmethod
    def is_good_gga(gga, bad_data_quality=None):
        if not gga or gga == '':
            return False
        if not gga.startswith('$GPGGA'):
            return False
        data_quality = gga.split(',')[6]
        if data_quality == '':
            return False
        if bad_data_quality and data_quality in bad_data_quality:
            return False
        return True

