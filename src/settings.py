#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from land.msg import *
from land.srv import *


class settings:
    def __init__(self):
        self.param_service = rospy.Service('gps_odom_switch', position_config, self.config)
        self.pub_config = rospy.Publisher("gps_camera_flag", Flag, queue_size=1)

    def config(self, msg):
        self.pub_config.publish(msg.switch_param)


if __name__ == '__main__':
    rospy.init_node('settings')

    opfix = settings()
    rospy.spin()
