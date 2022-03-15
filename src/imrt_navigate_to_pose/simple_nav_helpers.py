#!/usr/bin/env python

# Copyright (c) 2022 Fetullah Atas, Norwegian University of Life Sciences
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
import math
import tf.transformations

import sys
sys.dont_write_bytecode = True


class SimpleNavHelpers():
    def __init__(self, *args):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def pose_euclidean_dist(self, a, b):

        return math.sqrt((a.position.x - b.position.x) ** 2 +
                         (a.position.y - b.position.y) ** 2 +
                         (a.position.z - b.position.z) ** 2)

    def get_curr_robot_pose(self):
        curr_robot_pose = PoseStamped()
        curr_robot_pose.header.frame_id = "odom"
        curr_robot_pose.header.stamp = rospy.Time().now()
        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_link", rospy.Time().now(), rospy.Duration(1.0))
            curr_robot_pose.pose.position.x = transform.transform.translation.x
            curr_robot_pose.pose.position.y = transform.transform.translation.y
            curr_robot_pose.pose.position.z = transform.transform.translation.z
            curr_robot_pose.pose.orientation = transform.transform.rotation

        except (tf2_ros.TypeException, tf2_ros.NotImplementedException):
            rospy.loginfo("Failed to get current robot pose")
        return curr_robot_pose

    def clip(self, val, min_, max_):
        return min_ if val < min_ else max_ if val > max_ else val


class PurePursuitController():
    def __init__(self, linear_k, angular_k, linear_max, angular_max):
        self.linear_k = linear_k
        self.angular_k = angular_k
        self.linear_max = linear_max
        self.angular_max = angular_max
        self.simple_nav_helper = SimpleNavHelpers()

    def compute_error(self, curr_robot_pose,  curr_goal_pose, dist_to_goal_satisfied):

        robot_quat_exp = [curr_robot_pose.pose.orientation.x, curr_robot_pose.pose.orientation.y,
                          curr_robot_pose.pose.orientation.z, curr_robot_pose.pose.orientation.w]
        robot_euler = tf.transformations.euler_from_quaternion(
            robot_quat_exp)

        goal_quat_exp = [curr_goal_pose.pose.orientation.x, curr_goal_pose.pose.orientation.y,
                         curr_goal_pose.pose.orientation.z, curr_goal_pose.pose.orientation.w]
        goal_euler = tf.transformations.euler_from_quaternion(
            goal_quat_exp)

        robot_roll, robot_pitch, robot_yaw = robot_euler[0], robot_euler[1], robot_euler[2]
        goal_roll, goal_pitch, goal_yaw = goal_euler[0], goal_euler[1], goal_euler[2]

        err_local = [curr_goal_pose.pose.position.x - curr_robot_pose.pose.position.x,
                     curr_goal_pose.pose.position.y - curr_robot_pose.pose.position.y,
                     robot_yaw - goal_yaw]

        if dist_to_goal_satisfied:
            rot_error = goal_yaw - robot_yaw
        else:
            rot_error = math.atan2(err_local[1], err_local[0]) - robot_yaw

        dist_error = math.sqrt(err_local[0]**2 + err_local[1]**2)

        return dist_error, rot_error

    def compute_velocities(self, curr_robot_pose,  curr_goal_pose, dist_to_goal_satisfied):

        robot_quat_exp = [curr_robot_pose.pose.orientation.x, curr_robot_pose.pose.orientation.y,
                          curr_robot_pose.pose.orientation.z, curr_robot_pose.pose.orientation.w]
        robot_euler = tf.transformations.euler_from_quaternion(
            robot_quat_exp)

        goal_quat_exp = [curr_goal_pose.pose.orientation.x, curr_goal_pose.pose.orientation.y,
                         curr_goal_pose.pose.orientation.z, curr_goal_pose.pose.orientation.w]
        goal_euler = tf.transformations.euler_from_quaternion(
            goal_quat_exp)

        robot_roll, robot_pitch, robot_yaw = robot_euler[0], robot_euler[1], robot_euler[2]
        goal_roll, goal_pitch, goal_yaw = goal_euler[0], goal_euler[1], goal_euler[2]

        err_local = [curr_goal_pose.pose.position.x - curr_robot_pose.pose.position.x,
                     curr_goal_pose.pose.position.y - curr_robot_pose.pose.position.y,
                     robot_yaw - goal_yaw]

        k1 = self.linear_k
        k2 = self.angular_k
        max_v = self.linear_max
        max_w = self.angular_max

        v_in = k1 * math.sqrt(err_local[0]**2 + err_local[1]**2)

        if dist_to_goal_satisfied:
            w_in = k2 * (goal_yaw - robot_yaw)
        else:
            w_in = k2 * (math.atan2(err_local[1], err_local[0]) - robot_yaw)

        v_in = self.simple_nav_helper.clip(
            v_in, -self.linear_max, self.linear_max)
        w_in = self.simple_nav_helper.clip(
            w_in, -self.angular_max, self.angular_max)

        return v_in, w_in
