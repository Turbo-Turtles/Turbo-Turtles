#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nav_msgs.msg import Odometry

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from turtlebot3_interfaces.msg import Mission
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32

# classes start

class PositionListener(Node):
    def __init__(self):
        super().__init__("tf2_frame_listener")

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 0.0

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
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.z = msg.pose.pose.orientation.z
        self.w = msg.pose.pose.orientation.w

    def get_position(self):
        return self.x, self.y, self.z, self.w

# class GetMinDist(Node):
#     def __init__(self):
#         super().__init__("get_min_range")

#         self.min_dist = 100.0

#         qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
#                                           history=rclpy.qos.HistoryPolicy.KEEP_LAST,
#                                           depth=1)
        
#         self.odom_sub_ = self.create_subscription(
#             LaserScan,
#             'scan',
#             self.listener_callback,
#             qos_profile=qos_policy,
#         )

#         self.min_dist_pub_ = self.create_publisher(
#             Float32,
#             'min_dist',
#             1
#         )

#         self.timer_ = self.create_timer(0.5, self.timer_callback)

#     def listener_callback(self, msg):
#         for item in msg.ranges:
#             if item != 0.0 and item < self.min_dist:
#                 self.min_dist = item

#     def timer_callback(self):


#     def get_min_dist(self):
#         return self.min_dist


class TunnelMission(Node):
    def __init__(self):
        super().__init__("tunnel_mission")

        self.active = False

        # initial coordinates, to base the goal location of
        self.x, self.y, self.z, self.w = self.get_location()

        # create goal location
        # self.goal_dist = self.get_dist(225)
        self.goal_x = self.x - 0.5
        self.goal_y = self.y
        self.goal_z = self.z
        self.goal_w = self.w

        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create navigator
        self.navigator_ = BasicNavigator()

        # set initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
        initial_pose.pose.position.x = self.x
        initial_pose.pose.position.y = self.y
        initial_pose.pose.orientation.z = self.z
        initial_pose.pose.orientation.w = self.w
        self.navigator_.setInitialPose(initial_pose)

        self.create_subscription(
            Mission,
            'mission',
            self.mission_callback,
            qos_profile=qos_policy
        )

    def mission_callback(self, msg):
        if msg.mission_name == "tunnel":
            self.active = True

            # get current location
            x, y, z, w = self.get_location()

            # set initial pose
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
            tunnel_entry = PoseStamped()
            tunnel_entry.header.frame_id = 'map'
            tunnel_entry.header.stamp = self.navigator_.get_clock().now().to_msg()
            tunnel_entry.pose.position.x = x
            tunnel_entry.pose.position.y = y - 2* 0.3
            tunnel_entry.pose.orientation.z = z
            tunnel_entry.pose.orientation.w = w
            goal_poses.append(tunnel_entry)

            # create tunnel exit waypoint
            tunnel_exit = PoseStamped()
            tunnel_exit.header.frame_id = 'map'
            tunnel_exit.header.stamp = self.navigator_.get_clock().now().to_msg()
            tunnel_exit.pose.position.x = self.goal_x
            tunnel_exit.pose.position.y = self.goal_y
            tunnel_exit.pose.orientation.z = self.goal_z
            tunnel_exit.pose.orientation.w = self.goal_w
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
        x, y, z, w = get_current_pose.get_position()
        get_current_pose.destroy_node()

        return x, y, z, w
    
    # def get_dist(self, area):
    #     # get distances for tunnel mission goals
    #     get_dist = GetMinDist(area)
    #     rclpy.spin_once(get_dist)
    #     min_dist = get_dist.get_min_dist()
    #     get_dist.destroy_node()

    #     self.get_logger().info('The minimal distance: {0}'.format(min_dist))

    #     return min_dist

# classes end

# functions start

# functions end

# main

def main():
    rclpy.init()
    node = TunnelMission()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()