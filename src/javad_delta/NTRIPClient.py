#!/usr/bin/env python2

import rospy
from nmea_msgs.msg import Sentence

from serial import Serial, SerialException

from threading import Thread, Lock
from httplib import HTTPConnection
from base64 import b64encode


class NTRIPClient:
    def __init__(self, ntrip_configs, serial_configs):
        # GGA
        self.serial_configs = serial_configs
        self.gga_lock = Lock()
        self.latest_gga = ''
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
        self.rtcm_thread = Thread(self.rtcm_thread_func)
        self.rtcm_thread.start()

    def publish_gga_sentence(self):
        # Read GGA sentence
        msg = Sentence()
        msg.header.frame_id = 'gps_origin'
        msg.header.seq = self.seq
        self.seq += 1
        msg.header.stamp = rospy.Time.now()
        msg.sentence = self.read_gga_sentence()
        if not msg.sentence:
            return
        elif not msg.sentence.startswith('$GPGGA'):
            return
        self.nmea_pub.publish(msg)

    def read_gga_sentence(self):
        gga = self.leftover_gga

        while not rospy.is_shutdown():
            with self.serial_lock:
                data = self.serial.read(128)
            pos = data.find('\r\n')
            if pos != -1:
                gga += data[:pos]
                self.leftover_gga = data[pos + 2:]
                with self.gga_lock:
                    self.latest_gga = gga
                return gga
            else:
                gga += data

    def rtcm_thread_func(self):
        while not rospy.is_shutdown():
            with self.gga_lock:
                gga = self.latest_gga
            # Send to NTRIP server
            self.connection.request('GET',
                                    '/' + self.ntrip_configs['stream'],
                                    gga,
                                    self.header)
            try:
                response = self.connection.getresponse()
            except Exception as e:
                rospy.logerr('Exception when connecting to NTRIP server')

            if response.status != 200:
                err = 'NTRIP Server response: {0}'.format(response.status)
                raise Exception(err)

            rtcm, rtcm_leftover = NTRIPClient.read_rtcm(response)
            # Write to GNSS receiver
            with self.serial_lock:
                self.serial.write(rtcm)

    @staticmethod
    def read_rtcm(response):
        rtcm = ''
        # Keep reading RTCM, several bytes at a time
        # TODO: response.read()
        while not rospy.is_shutdown():
            data = response.read()
            pos = data.find('\r\n')
            if pos != -1:
                rtcm += data[:pos]
                rtcm_leftover = data[pos + 2:]
                return rtcm, rtcm_leftover
            else:
                rtcm += data
