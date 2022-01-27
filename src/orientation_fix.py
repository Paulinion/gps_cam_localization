#!/usr/bin/python
# -*- coding: utf-8 -*-
import math
import tf
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import QuaternionStamped
from land.srv import *
from sensor_msgs.msg import NavSatFix
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tf.transformations import quaternion_from_euler


class orientation_fix:
    def __init__(self):
        rospy.Subscriber('heading', QuaternionStamped, self.heading_callback)
        self.heading_ = None
        rospy.Subscriber('/camera/odom/sample', Odometry, self.odometry)
        self.err = 0
        self.Heading_flag = 3
        self.tf_broadcaster_ = tf.TransformBroadcaster()

        self.pub = rospy.Publisher('fixed_heading', Quaternion, queue_size=1)
        self.update_timer = rospy.Timer(rospy.Duration(1), self.fix_heading)
        self.prev_angle = None
        self.theta_add = 0

    def odometry(self, msg):
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        self.quaternion = msg.pose.pose.orientation
        self.angle_param = 0
        self.camera_x_fixed = (self.pose_x) * math.cos(self.angle_param) - (
            self.pose_y) * math.sin(self.angle_param)
        self.camera_y_fixed = (self.pose_x) * math.sin(self.angle_param) + (
            self.pose_y) * math.cos(self.angle_param)
        camera_euler = euler_from_quaternion(
            [self.quaternion.x, self.quaternion.y, self.quaternion.z, self.quaternion.w])
        self.camera_z = camera_euler[2]

    def heading_callback(self, msg):
        self.heading_ = msg.quaternion

    def fix_angle(self, theta):

        if theta > 2 * math.pi:
            theta = theta - 2 * math.pi
        elif theta < 0:
            theta = 2 * math.pi + theta
        # print(theta, "fixed_")

        return theta

    def fix_heading(self, _):
        print(self.heading_, "bug_fix")
        if self.heading_ is None or math.isnan(self.heading_.x):
            return

        (roll, pitch, theta) = \
            euler_from_quaternion([self.heading_.x,
                                   self.heading_.y,
                                   self.heading_.z,
                                   self.heading_.w])

        theta_copy = theta
        theta += math.pi

        if self.prev_angle is None:
            self.prev_angle = theta_copy
        self.prev_angle += math.pi

        if self.prev_angle > theta:
            self.err += abs(self.prev_angle - theta)
        else:
            self.err -= abs(self.prev_angle - theta)
        self.err = (self.err + math.pi) % (2 * math.pi) - math.pi
        theta = self.fix_angle(theta + 2 * self.err - math.pi / 2.4)
        theta -= math.pi

        self.prev_angle = theta_copy

        self.camera_z = self.fix_angle(self.camera_z + math.pi / 2 - math.pi / 1.4 + math.pi) - math.pi
        print(theta, "final", self.camera_z, "camera z")
        self.tf_broadcaster_.sendTransform(
            # (self.camera_x_fixed, self.camera_y_fixed, 0),
            (0, 0, 0),
            quaternion_from_euler(0, 0, self.camera_z),
            rospy.Time.now(),
            "camera", "gps_info"
        )

        self.fixed_heading = Quaternion

        dif = theta - self.camera_z
        if abs(dif) > math.pi:
            if dif > 0:
                dif = dif - 2 * math.pi
            else:
                dif = dif + 2 * math.pi
        print(dif, "Error in heading")
        self.fixed_heading = quaternion_from_euler(roll, pitch, dif)
        self.pub.publish(self.fixed_heading[0], self.fixed_heading[1], self.fixed_heading[2], self.fixed_heading[3])


if __name__ == '__main__':
    rospy.init_node('orientation_fix')

    opfix = orientation_fix()
    rospy.spin()
