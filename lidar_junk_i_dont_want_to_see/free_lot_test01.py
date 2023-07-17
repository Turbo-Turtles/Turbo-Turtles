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
        self.free_slot = None

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

        self.create_timer(
            0.5,
            self.timer_callback
        )

    def sub_callback(self, msg):
        ranges = msg.ranges

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
        else:
            print("error")

    def get_free(self):
        return self.free_slot
    
    def timer_callback(self):
        print(self.get_free())
    

# main
def main():
    rclpy.init()
    node = GetFreeSlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()