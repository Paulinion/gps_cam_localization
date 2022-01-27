#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from land.srv import *

def gps_odom_switch(arg1):
    rospy.wait_for_service('gps_odom_switch')
    try:
        param_service = rospy.ServiceProxy('gps_odom_switch', position_config)
        resp1 = param_service(arg1)
        return str(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [gps_odom_switch]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        switch_param = int(sys.argv[1])

    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s"% switch_param)
    gps_odom_switch(switch_param)