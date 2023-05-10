#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
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

from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nav_msgs.msg import Odometry

import rclpy
from rclpy.time import Time
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class FrameListener(Node):
    def __init__(self):
        super().__init__("tf2_frame_listener")

        self.x = 0
        self.y = 0
        self.frame_dist = 0

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_sub_ = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            qos_profile=qos_policy,
        )

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.timer_callback)

    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def timer_callback(self):
        self.get_frame_dist()

        print("x:", self.x, "y:", self.y, "\n")
        print("transform:", self.frame_dist.transform.translation.x, "|", self.frame_dist.transform.translation.y)

    def get_frame_dist(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = 'base_link'
        to_frame_rel = 'odom'

        # Look up for the transformation between frames
        self.frame_dist = self.tf_buffer.lookup_transform(
            to_frame_rel,
            from_frame_rel,
            Time())

def main():
    rclpy.init()
    node = FrameListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()