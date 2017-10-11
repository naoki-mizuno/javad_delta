#!/usr/bin/env python2

import rospy

import javad_delta

rospy.init_node('javad_delta_node')

ntrip_configs = {
    'server': rospy.get_param('~ntrip_server'),
    'stream': rospy.get_param('~ntrip_stream'),
    'port': rospy.get_param('~ntrip_port'),
    'user': rospy.get_param('~ntrip_user'),
    'password': rospy.get_param('~ntrip_password'),
}
serial_configs = {
    'port': rospy.get_param('~serial_port'),
    'baud': rospy.get_param('~serial_baud'),
}

client = javad_delta.NTRIPClient(ntrip_configs, serial_configs)

while not rospy.is_shutdown():
    client.publish_gga_sentence()
client.stop = True
