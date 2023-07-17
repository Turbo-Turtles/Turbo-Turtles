#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped

from math import acos, sin, cos


def get_relative_coords(self, pose: PoseStamped, rel_x, rel_y, rel_angle = 0.0):
    rel_pose = PoseStamped()

    angle = self.get_angle(pose.pose.orientation.z, pose.pose.orientation.w)
    z, w = self.get_quaternion(angle + rel_angle)

    rel_pose.header.frame_id = 'map'
    rel_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
    rel_pose.pose.position.x = pose.pose.position.x + rel_x * cos(angle / 57.3) + rel_y * sin(angle / 57.3)
    rel_pose.pose.position.y = pose.pose.position.y + rel_x * sin(angle / 57.3) + rel_y * cos(angle / 57.3)
    rel_pose.pose.orientation.z = z
    rel_pose.pose.orientation.w = w

    return rel_pose

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
