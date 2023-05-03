from nav2_simple_commander.robot_navigator import BasicNavigator
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
import rclpy

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

    path = nav2.getPath(init_pose, goal_pose, planner_id="GridBased")
    smoothed_path = nav2.smoothPath(path)

    nav2.goToPose(goal_pose)

    while not nav2.isTaskComplete():
        feedback = nav2.getFeedback()
        if feedback.navigation_duration > 600:
            nav2.cancelTask()

    result = nav2.getResult()

    if result.SUCCEEDED:
        print("Goal succeeded!")
    elif result.CANCELED:
        print("Goal was canceled!")
    elif result.FAILED:
        print("Goal failed!")

    nav2.lifecycleShutdown()
    rclpy.shutdown()