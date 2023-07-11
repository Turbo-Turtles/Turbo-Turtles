#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from math import acos, sin, cos

from lidar_navigation.position_listener import PositionListener

class Angle(Node):
    def __init__(self):
        super().__init__("angle_listener")

        self.create_timer(
            0.5,
            self.timer_callback
        )

    def timer_callback(self):
        x, y, z, w = self.get_location()

        angle = self.get_angle(z, w)
        n_z, n_w = self.get_quaternion(angle+45)
        angle2 = self.get_angle(n_z, n_w)

        print(z, w, "\n", n_z, n_w, "\n")

        print(angle)
        print(angle2)
        print("\n")

    def get_angle(self, z, w):
        # 57.3 deg are 1 rad
        if z < 0:
            angle = 360 - acos(w) * 2 * 57.3
        else:
            angle = acos(w) * 2 * 57.3

        return angle
    
    def get_quaternion(self, angle):
        angle /= 57.3
        
        z = sin(angle/2.0)
        w = cos(angle/2.0)

        return z, w

    def get_location(self):
        # get current loaction
        get_current_pose = PositionListener()
        rclpy.spin_once(get_current_pose)
        position = get_current_pose.get_position()
        get_current_pose.destroy_node()

        return position.pose.position.x, position.pose.position.y, position.pose.orientation.z, position.pose.orientation.w
    
# main
def main():
    rclpy.init()
    node = Angle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()