#!/usr/bin/env python2

import rospy
from nmea_msgs.msg import Sentence

from serial import Serial, SerialException

from threading import Thread, Lock
import socket
from base64 import b64encode


class NTRIPClient:
    def __init__(self, ntrip_configs, serial_configs):
        self.__init_gga__(serial_configs)
        self.__init_rtcm__(ntrip_configs)

    def __init_gga__(self, serial_configs):
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

    def __init_rtcm__(self, ntrip_configs):
        # RTCM
        self.ntrip_configs = ntrip_configs
        user_pass = '{0}:{1}'.format(ntrip_configs['user'],
                                     ntrip_configs['password'])
        self.header = ''
        self.header += 'GET /{0} HTTP/1.0\r\n'.format(ntrip_configs['stream'])
        self.header += 'User-Agent: NTRIP ntrip_client\r\n'
        self.header += 'Connection: close\r\n'
        self.header += 'Accept: */*\r\n'
        self.header += 'Authorization: Basic {0}\r\n'.format(b64encode(user_pass))
        self.header += '\r\n'
        self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connection.settimeout(10)
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

    def connect_to_server(self, server, port):
        err_indicator = self.connection.connect_ex((server, int(port)))
        return err_indicator

    def rtcm_thread_func(self):
        while not rospy.is_shutdown():
            err = self.connect_to_server(self.ntrip_configs['server'],
                                         self.ntrip_configs['port'])
            if err != 0:
                rospy.logerr('Failed to connect to server: {0}'.format(err))
                rospy.sleep(1)
                continue

            status = None
            try:
                status = self.send_headers()
            except Exception:
                rospy.logerr('Error when sending headers')

            if status == 200:
                rospy.loginfo('Ready to receive RTCM')
                break

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
                self.connection.send(gga + '\r\n')
            except Exception as e:
                rospy.logerr('Failed to send GGA sentence')
                continue

            # Parse response
            try:
                read_bytes = self.read_rtcm()
                rospy.loginfo('Received RTCM ({0} bytes)'.format(read_bytes))
            except Exception:
                rospy.logerr('Failed to read RTCM')
                continue
        self.connection.close()

    def send_headers(self):
        self.connection.send(self.header)
        header_lines = self.connection.recv(4096).split('\r\n')
        for line in header_lines:
            if line.find("SOURCETABLE") >= 0:
                rospy.logerr("Mount point does not exist")
            elif line.find("401 Unauthorized") >= 0:
                return 401
            elif line.find("404 Not Found") >= 0:
                return 404
            elif line.find("ICY 200 OK") >= 0:
                return 200
            elif line.find("HTTP/1.0 200 OK") >= 0:
                return 200
            elif line.find("HTTP/1.1 200 OK") >= 0:
                return 200
        return -1

    def read_rtcm(self):
        # Keep reading RTCM and send it to the receiver
        read_bytes = 0
        while True:
            rtcm = self.connection.recv(256)
            # Write to GNSS receiver
            with self.serial_lock:
                sent = self.serial.write(rtcm)
                read_bytes += len(rtcm)

            with self.gga_lock:
                # Return if there's newer GGA
                if rospy.is_shutdown() or self.has_new_gga:
                    return read_bytes

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

