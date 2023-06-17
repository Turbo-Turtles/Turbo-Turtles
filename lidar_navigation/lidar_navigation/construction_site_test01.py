#! /usr/bin/env python3
# get own path
import os, sys
sys.path.append(os.path.join(
    os.path.dirname(__file__),
    "../../../../../../Turbo-Turtles/lidar_navigation/lidar_navigation/"))

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from turtlebot3_interfaces.msg import Mission

from map_recognition import MapRecognition
from position_listener import PositionListener

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# classes start
class ConstructionMission(Node):
    def __init__(self):
        super().__init__("construction_mission")

        # variable to avoid execution when already running
        self.active = False

        # quality of service policy
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create navigator
        self.navigator_ = BasicNavigator()

        self.create_subscription(
            Mission,
            'mission',
            self.mission_callback,
            qos_profile=qos_policy
        )

    def mission_callback(self, msg):
        if msg.mission_name == "construction" and self.active == False:
            self.active = True

            # current position
            x, y, z, w = self.get_location()

            # set initial pose with current position
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
            initial_pose.pose.position.x = x
            initial_pose.pose.position.y = y
            initial_pose.pose.orientation.z = z
            initial_pose.pose.orientation.w = w
            self.navigator_.setInitialPose(initial_pose)

            # set goal posees to follow
            goal_poses = []

##############################
#
# how we gonna do dis
#
            
#
###################################

            nav_start = self.navigator_.get_clock().now()
            self.navigator_.followWaypoints(goal_poses)

            i = 0
            while not self.navigator_.isTaskComplete():
                ################################################
                #
                # Implement some code here for your application!
                #
                ################################################

                # Do something with the feedback
                i = i + 1
                feedback = self.navigator_.getFeedback()
                if feedback and i % 5 == 0:
                    print('Executing current waypoint: ' +
                        str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                    now = self.navigator_.get_clock().now()

                    # Some navigation timeout to demo cancellation
                    if now - nav_start > Duration(seconds=600.0):
                        self.navigator_.cancelTask()
            
            # Do something depending on the return code
            result = self.navigator_.getResult()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
            elif result == TaskResult.FAILED:
                print('Goal failed!')
            else:
                print('Goal has an invalid return status!')

            self.navigator_.lifecycleShutdown()

    
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
    node = ConstructionMission()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()