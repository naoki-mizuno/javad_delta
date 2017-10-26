#!/usr/bin/env python2

import rospy
from nmea_msgs.msg import Sentence
import tf

import javad_delta


rospy.init_node('javad_delta_node')

ros_configs = {
    'publish_tf': rospy.get_param('publish_tf', True),
    'gps_origin_frame': rospy.get_param('gps_origin_frame', 'gps_origin'),
    'gps_antenna_frame': rospy.get_param('gps_antenna_frame', 'gps_antenna'),
    # Required data quality to publish tf
    'gps_required_quality': rospy.get_param('gps_required_quality', ['4', '5']),
    # Uses WGS84 by default
    'gps_src_epsg': rospy.get_param('gps_src_epsg', 4326),
    # Earth Centered, Earth Fixed
    'gps_tgt_epsg': rospy.get_param('gps_tgt_epsg', 4978),
}
ntrip_configs = {
    'enable': rospy.get_param('~ntrip_enable'),
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

nmea_pub = rospy.Publisher('nmea_sentence',
                           Sentence,
                           queue_size=100)
seq = 0


def publish_gga(gga, configs):
    global nmea_pub
    global seq

    msg = Sentence()
    msg.header.frame_id = configs['gps_origin_frame']
    msg.header.seq = seq
    seq += 1
    msg.header.stamp = rospy.Time.now()
    msg.sentence = gga
    if javad_delta.GGA.is_good_gga(msg.sentence):
        nmea_pub.publish(msg)
    return msg


def send_transform(msg, configs):
    fix = javad_delta.GGA(msg)
    transformer = javad_delta.CoordTransformer(configs['gps_src_epsg'],
                                               configs['gps_tgt_epsg'])
    x, y, z = transformer.transform(fix)
    translation = (x, y, z)
    # No orientation
    q = tf.transformations.quaternion_from_euler(0, 0, 0)
    rotation = (q.x, q.y, q.z, q.w)
    tf_b = tf.TransformBroadcaster()
    tf_b.sendTransform(translation,
                       rotation,
                       msg.header.stamp,
                       configs['gps_antenna_frame'],
                       configs['gps_origin_frame'])


client = javad_delta.NTRIPClient(ntrip_configs, serial_configs)

while not rospy.is_shutdown():
    gga_sentence = client.read_gga_sentence()
    published_msg = publish_gga(gga_sentence, ros_configs)
    is_good_gaa = javad_delta.GGA.is_good_gga(gga_sentence,
                                              ros_configs['gps_required_quality'])
    if ros_configs['publish_tf'] and is_good_gaa:
        send_transform(published_msg, ros_configs)
