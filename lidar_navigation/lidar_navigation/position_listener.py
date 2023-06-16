#! /usr/bin/env python3
##################################
#
# Returns the current position of the robot
#
# ros2 service call /current_position turtlebot3_interfaces/srv/GetPosition "{request: True}"
#
##################################

import sys
sys.path.insert(0, '')

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
# from turtlebot3_interfaces.srv import GetPosition

# classes start

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

        # self.position_service_ = self.create_service(
        #     GetPosition,
        #     'current_position',
        #     self.service_callback,
        # )

    def listener_callback(self, msg):
        self.pos.pose.position.x = msg.pose.pose.position.x
        self.pos.pose.position.y = msg.pose.pose.position.y
        self.pos.pose.orientation.z = msg.pose.pose.orientation.z
        self.pos.pose.orientation.w = msg.pose.pose.orientation.w

        # print(self.pos.pose.position.x)
        # print(self.pos.pose.position.y)

    # def service_callback(self, request: GetPosition.Request, response: GetPosition.Response):
    #     self.get_logger().info("Sending current position...")
    #     response.position = self.pos

    #     print(self.pos.pose.position.x)
    #     print(self.pos.pose.position.y)
        
    #     return response

    def get_position(self):
        return self.pos


# main
# def main():
#     rclpy.init()
#     node = PositionListener()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

#     exit(0)


# if __name__ == '__main__':
#     main()