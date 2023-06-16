#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlebot3_interfaces.srv import MinRange

class TunnelMission(Node):
    def __init__(self):
        super().__init__("tunnel_mission")

        self.request_ = MinRange.Request()

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.client_ = self.create_client(
            MinRange,
            'get_min_range'
        )

        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.timer_ = self.create_timer(1.0, self.timer_callback)
            

    def timer_callback(self):
        min_range = self.get_min_range(135)
        self.get_logger().info('minimal distance from {0} to {1} => {2}'.format(135, 135+90, min_range))

    def get_min_range(self, area):
        self.request_.area = area
        self.future = self.client_.call_async(self.request_)
        rclpy.spin_until_future_complete(self, self.future)

        return self.future

def main():
    rclpy.init()
    node = TunnelMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()