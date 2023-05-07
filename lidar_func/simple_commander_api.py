from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult, Duration
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import rclpy

"""
simulation bringup/launch

ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

---------------------------------------------------------
possible navigation launches

ros2 launch turtlebot3_navigation2 navigation2.launch.py


ros2 launch slam_toolbox online_async_launch.py

ros2 launch nav2_bringup slam_launch.py
ros2 launch turtlebot3_cartographer cartographer.launch.py
ros2 launch nav2_bringup rviz_launch.py
ros2 launch nav2_bringup navigation_launch.py
"""

init_pose = PoseStamped()
init_pose.pose.position.x = 0.0
init_pose.pose.position.y = 0.0
goal_pose = PoseStamped()
goal_pose.pose.position.x = 1.0
goal_pose.pose.position.x = 0.0

def main(args=None):
    rclpy.init(args=args)

    nav2 = BasicNavigator()
    
    nav2.setInitialPose(init_pose)
    nav2.lifecycleStartup()

    #path = nav2.getPath(init_pose, goal_pose, planner_id="GridBased")
    #smoothed_path = nav2.smoothPath()

    nav2.goToPose(goal_pose)

    while not nav2.isTaskComplete():
        feedback = nav2.getFeedback()
        if feedback.navigation_time > 600:
            nav2.cancelTask()

    result = nav2.getResult()

    if result == TaskResult.SUCCEEDED:
        print("Goal succeeded!")
    elif result == TaskResult.CANCELED:
        print("Goal was canceled!")
    elif result == TaskResult.FAILED:
        print("Goal failed!")

    nav2.lifecycleShutdown()
    rclpy.shutdown()