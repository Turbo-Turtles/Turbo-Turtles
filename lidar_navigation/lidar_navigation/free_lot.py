# import via:
# from lidar_navigation.free_lot import GetFreeSlot

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan


class GetFreeSlot(Node):
    def __init__(self):
        super().__init__("get_free_parking_lot")

        # lower and upper end need tuning
        self.lower_end = 0.1
        self.upper_end = 0.4
        self.free_slot = "right"

        # quality of service policy
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.create_subscription(
            LaserScan,
            'scan',
            self.sub_callback,
            qos_profile=qos_policy
        )

    def sub_callback(self, msg):
        # extract the ray lenghts
        ranges = msg.ranges

        # clean up the data (distances lower than robot radius are impossible/unlikely)
        for i in range(len(ranges)):
            if ranges[i] <= 0.09:
                ranges[i] = 4.0

        # check left side
        left = 0
        for item in ranges[:180]:
            if item > self.lower_end and item < self.upper_end:
                left += 1

        # check right side
        right = 0
        for item in ranges[181:]:
            if item > self.lower_end and item < self.upper_end:
                right += 1

        # compare sides
        if left < right:
            self.free_slot = "left"
        elif left > right:
            self.free_slot = "right"

    def get_free(self):
        # return the free parking lot
        return self.free_slot