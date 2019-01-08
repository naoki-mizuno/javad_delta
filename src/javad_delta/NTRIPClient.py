from GGAParser import GGA

import rospy

from serial import Serial, SerialException

from threading import Thread, Lock
import socket
from base64 import b64encode


class NTRIPClient:
    def __init__(self, ntrip_configs, serial_configs):
        self.__init_serial__(serial_configs)
        self.__init_ntrip__(ntrip_configs)

    def __init_serial__(self, serial_configs):
        # GGA
        self.serial_configs = serial_configs
        self.gga_lock = Lock()
        self.latest_gga = ''
        self.has_new_gga = False
        self.leftover_gga = ''
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

    def __init_ntrip__(self, ntrip_configs):
        if not ntrip_configs['enable']:
            return

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
        while not rospy.is_shutdown():
            try:
                err = self.connection.connect_ex((server, int(port)))
            except Exception:
                err = None

            if err != 0:
                rospy.logerr('Retrying connection in 3 seconds: {0}'.format(server))
                self.connection.close()
                rospy.sleep(3)
                self.connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.connection.settimeout(10)
                continue

            status = None
            try:
                status = self.send_headers()
            except Exception:
                rospy.logerr('Error when sending headers')

            if status == 200:
                rospy.loginfo('Ready to receive RTCM')
                break

    def rtcm_thread_func(self):
        self.connect_to_server(self.ntrip_configs['server'],
                               self.ntrip_configs['port'])

        # Get RTCM data
        while not rospy.is_shutdown():
            # Get latest GGA
            with self.gga_lock:
                gga = self.latest_gga
                self.has_new_gga = False
            if not GGA.is_good_gga(gga, bad_data_quality=['0']):
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
        rospy.loginfo('Closed connection')

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
        # Keep reading RTCM and send it to the receiver until new GGA has arrived
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
