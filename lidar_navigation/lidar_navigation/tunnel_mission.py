#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from turtlebot3_interfaces.msg import Mission, Progress

from lidar_navigation.position_listener import PositionListener

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from math import cos, acos, sin


# class
class TunnelMission(Node):
    def __init__(self):
        super().__init__("tunnel_mission")

        self.active = False

        # initial coordinates, to base the goal location of
        self.x, self.y, self.z, self.w = self.get_location()

        # quality of service policy
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create navigator
        self.navigator_ = BasicNavigator()

        # start pose (, to later base the goal location of)
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = 'map'
        self.start_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
        self.start_pose.pose.position.x = self.x
        self.start_pose.pose.position.y = self.y
        self.start_pose.pose.orientation.z = self.z
        self.start_pose.pose.orientation.w = self.w

        # publisher
        self.pub_progress = self.create_publisher(
            Progress,
            'progress',
            1
        )

        # subscriber
        self.mission_sub_ = self.create_subscription(
            Mission,
            'mission',
            self.mission_callback,
            qos_profile=qos_policy
        )

    def mission_callback(self, msg):
        if msg.mission_name == "tunnel" and self.active == False:
            self.active = True

            # get current location
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

            # create tunnel entry waypoint
            # tunnel_dist = self.get_dist(315)
            tunnel_entry = self.get_relative_coords(initial_pose, 0.6, 0)
            goal_poses.append(tunnel_entry)

            # create tunnel exit waypoint
            tunnel_exit = self.get_relative_coords(self.start_pose, -0.5, 0)
            goal_poses.append(tunnel_exit)

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
            progress_msg = Progress()
            if result == TaskResult.SUCCEEDED:
                print('Goal succeeded!')
                progress_msg.sender = "nav2"
                progress_msg.state = 1
            elif result == TaskResult.CANCELED:
                print('Goal was canceled!')
                progress_msg.sender = "nav2"
                progress_msg.state = 0
            elif result == TaskResult.FAILED:
                print('Goal failed!')
                progress_msg.sender = "nav2"
                progress_msg.state = 0
            else:
                print('Goal has an invalid return status!')
                progress_msg.sender = "nav2"
                progress_msg.state = 0

            self.pub_progress(progress_msg)

            self.navigator_.lifecycleShutdown()


    def get_relative_coords(self, pose: PoseStamped, rel_x, rel_y, rel_angle = 0.0):
        rel_pose = PoseStamped()

        angle = self.get_angle(pose.pose.orientation.z, pose.pose.orientation.w)
        z, w = self.get_quaternion(angle + rel_angle)

        rel_pose.header.frame_id = 'map'
        rel_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
        rel_pose.pose.position.x = pose.pose.position.x + rel_x * cos(angle / 57.3) + rel_y * cos((angle + 90) / 57.3)
        rel_pose.pose.position.y = pose.pose.position.y + rel_x * sin(angle / 57.3) + rel_y * sin((angle + 90) / 57.3)
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
    node = TunnelMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()