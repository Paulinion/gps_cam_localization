#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import tf
import tf2_ros
import rospy
from land.msg import *
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from land.srv import *
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped
from swri_transform_util.wgs84_transformer import Wgs84Transformer


class no_gps_orientation:
    def __init__(self):
        rospy.Subscriber('fix', NavSatFix, self.fix_callback)
        self.tf_broadcaster_ = tf.TransformBroadcaster()
        self.StartBroadcaster = tf2_ros.StaticTransformBroadcaster()
        rospy.Subscriber('/camera/odom/sample', Odometry, self.odometry_callback)
        rospy.Subscriber('gps_camera_flag', Flag, self.flag)

        self.fix_ = None
        self.odometry_ = None
        rospy.Subscriber('fixed_heading', Quaternion, self.quat)

        self.No_gps_flag = 0
        self.start_flag = 0
        self.zero_x, self.zero_y, self.zero_x_gps, self.zero_y_gps = [0] * 4

        rospy.Subscriber('local_xy_origin', PoseStamped, self.origin_callback)
        self.wgs84_transformer_ = None

        self.update_timer = rospy.Timer(rospy.Duration(0.5), self.no_gps_orientation)

    def quat(self, msg):

        self.fixed_heading = msg

    def flag(self, msg):
        self.No_gps_flag = msg.flag
        if self.No_gps_flag == 0 and self.start_flag == 1:
            self.start_flag = 0
            self.tf_broadcaster_.sendTransform(
                (0, 0, 0),
                quaternion_from_euler(0, 0, 0),
                rospy.Time.now(),
                "base_link", "gps_info"
            )


    def odometry_callback(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.quaternion = msg.pose.pose.orientation

    def origin_callback(self, msg):
        self.wgs84_transformer_ = Wgs84Transformer(msg)

    def fix_callback(self, msg):
        self.fix_ = msg
        self.gps_xy("")

    def gps_xy(self, _):
        self.gps_x, self.gps_y = self.wgs84_transformer_.wgs84_to_local_xy(
            [self.fix_.latitude,
             self.fix_.longitude])

    def start_params(self, _):
        if self.start_flag == 1:
            return
        self.zero_x = self.pose_x
        self.zero_y = self.pose_y
        self.zero_x_gps = self.gps_x
        self.zero_y_gps = self.gps_y
        self.z = euler_from_quaternion(
            [self.fixed_heading.x, self.fixed_heading.y, self.fixed_heading.z, self.fixed_heading.w])[2]
        self.camera_z = euler_from_quaternion(
            [self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])[2]

        self.start_flag = 1

    def handle_pose(self, _):
        self.angle_param = -math.pi / 1.4 + self.z
        self.camera_x_fixed = (self.pose_x - self.zero_x) * math.cos(self.angle_param) - (
                self.pose_y - self.zero_y) * math.sin(self.angle_param)
        self.camera_y_fixed = (self.pose_x - self.zero_x) * math.sin(self.angle_param) + (
                self.pose_y - self.zero_y) * math.cos(self.angle_param)
        self.tf_broadcaster_.sendTransform(
            (self.camera_x_fixed, self.camera_y_fixed, 0),
            quaternion_from_euler(0, 0, 0),
            rospy.Time.now(),
            "base_link", "gps_info"
        )

        print(math.sqrt((self.camera_x_fixed - (self.gps_x - self.zero_x_gps)) ** 2 + (
                self.camera_y_fixed - (self.gps_y - self.zero_y_gps)) ** 2),
              "Distance error")



    def no_gps_orientation(self, _):

        if self.No_gps_flag == 0:
            return

        self.start_params(_)
        if self.start_flag == 1:
            self.handle_pose(_)


if __name__ == '__main__':
    rospy.init_node('no_gps_orientation')

    opfix = no_gps_orientation()
    rospy.spin()
