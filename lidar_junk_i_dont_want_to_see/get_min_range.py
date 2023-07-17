#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from turtlebot3_interfaces.srv import MinRange

class GetLaserScan(Node):
    def __init__(self):
        super().__init__("get_laser_scan")

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.sub_ = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile=qos_policy
        )

        self.pub_ = self.create_publisher(
            list,
            'ranges'
        )

    def listener_callback(self, msg):
        self.ranges = msg.ranges

class MinRangeService(Node):
    def __init__(self):
        super().__init__("min_range_service")
        self.get_logger().info('service starting...')

        self.ranges = []

        self.service_ = self.create_service(
            MinRange,
            'get_min_range',
            self.service_callback,
            #qos_profile=qos_policy
        )
    
    def service_callback(self, request, response):
        response.min_range = 100.0
    
        if request.area < 269:
            for item in self.ranges[request.area:request.area+90]:
                if item != 0.0 and item < self.min_range:
                    self.min_range = item
        else:
            for item in self.ranges[request.area:359]:
                if item != 0.0 and item < self.min_range:
                    self.min_range = item
            for item in self.ranges[:359-request.area]:
                if item != 0.0 and item < self.min_range:
                    self.min_range = item

        return response

    def get_ranges(self, msg):
        self.ranges = msg.ranges


def main():
    rclpy.init()
    service = MinRangeService()
    rclpy.spin(service)
    service.destroy_service()
    rclpy.shutdown()


if __name__ == '__main__':
    main()