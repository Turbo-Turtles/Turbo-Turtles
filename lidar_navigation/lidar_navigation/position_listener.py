#! /usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped


class PositionListener(Node):
    def __init__(self):
        super().__init__("tf2_frame_listener")

        self.pos = PoseStamped()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.odom_sub_ = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            qos_profile=qos_policy,
        )

    def listener_callback(self, msg):
        self.pos.pose.position.x = msg.pose.pose.position.x
        self.pos.pose.position.y = msg.pose.pose.position.y
        self.pos.pose.orientation.z = msg.pose.pose.orientation.z
        self.pos.pose.orientation.w = msg.pose.pose.orientation.w

    def get_position(self):
        return self.pos
