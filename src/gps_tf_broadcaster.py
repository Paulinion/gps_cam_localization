#!/usr/bin/python

import rospy
from land.srv import *
from land.msg import *
import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler
from swri_transform_util.wgs84_transformer import Wgs84Transformer


class TFBroadcaster:
    def __init__(self):
        self.tf_broadcaster_ = tf.TransformBroadcaster()
        self.wgs84_transformer_ = None
        # odom_publisher_ = rospy.Publisher('odom', Odometry, queue_size=1)

        self.fix_msg_ = None
        rospy.Subscriber('gps_camera_flag', Flag, self.flag)

        self.No_gps_flag = 0
        self.flag = 0
        rospy.Subscriber('fix', NavSatFix, self.fix_callback)

        rospy.Subscriber('local_xy_origin', PoseStamped, self.origin_callback)

        self.update_timer = rospy.Timer(rospy.Duration(0.01), self.handle_pose)

    def flag(self, msg):
        self.No_gps_flag = msg.flag

        return ("gps_tf")

    def fix_callback(self, msg):
        self.fix_msg_ = msg



    def origin_callback(self, msg):
        self.wgs84_transformer_ = Wgs84Transformer(msg)

    def handle_pose(self, _):
        if (
                self.fix_msg_ is None or self.wgs84_transformer_ is None  or self.No_gps_flag == 1):
            return
        # print(self.No_gps_flag)

        self.fix = self.wgs84_transformer_.wgs84_to_local_xy(
            [self.fix_msg_.latitude,
             self.fix_msg_.longitude])
        z = self.fix_msg_.altitude

        self.tf_broadcaster_.sendTransform((self.fix[0], self.fix[1], z),
                                           quaternion_from_euler(0, 0, 0),
                                           rospy.Time.now(),
                                           "gps_info",
                                           "map")


if __name__ == '__main__':
    rospy.init_node('land_tf_broadcaster')

    br = TFBroadcaster()
    rospy.spin()
