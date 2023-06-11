#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import PoseStamped
from turtlebot3_interfaces.msg import Mission
from turtlebot3_interfaces.srv import GetPosition

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


# classes start
class TunnelMission(Node):
    def __init__(self):
        super().__init__("tunnel_mission")

        # variable to avoid execution when already running
        self.active = False

        # quality of service policy
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        # create navigator
        self.navigator_ = BasicNavigator()

        # start pose (, to later base the goal location of)
        self.start_pose = self.get_location()
        self.start_pose.header.frame_id = 'map'
        self.start_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
        self.navigator_.setInitialPose(self.start_pose)

        self.create_subscription(
            Mission,
            'mission',
            self.mission_callback,
            qos_profile=qos_policy
        )

        self.position_client_ = self.create_client(
            GetPosition,
            'current_position',
            qos_profile=qos_policy
        )

    def mission_callback(self, msg):
        if msg.mission_name == "tunnel" and self.active == False:
            self.active = True

            # set initial pose with current position
            initial_pose = self.get_location()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = self.navigator_.get_clock().now().to_msg()
            self.navigator_.setInitialPose(initial_pose)

            # set goal posees to follow
            goal_poses = []

            # create tunnel entry waypoint
            # tunnel_dist = self.get_dist(315)
            tunnel_entry = initial_pose
            tunnel_entry.header.stamp = self.navigator_.get_clock().now().to_msg()
            tunnel_entry.pose.position.y -= 0.6  # 0.6 meters infront of bot at tunnel entry
            goal_poses.append(tunnel_entry)

            # create tunnel exit waypoint
            tunnel_exit = self.start_pose
            tunnel_exit.header.stamp = self.navigator_.get_clock().now().to_msg()
            tunnel_exit.pose.position.x -= 0.5  # 0.5 meters behind bot from start point
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
        get_current_pose = self.position_client_.call_async(True)
        rclpy.spin_until_future_complete(self, get_current_pose)

        return get_current_pose.result()


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