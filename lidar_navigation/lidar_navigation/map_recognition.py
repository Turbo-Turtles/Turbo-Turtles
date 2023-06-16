#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration


from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PoseStamped
from turtlebot3_interfaces.msg import Mission
from turtlebot3_interfaces.srv import GetPosition

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class MapRecognition(Node):
    def __init__(self):
        super().__init__("map_recognition")

        # quality of service policy
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.map = OccupancyGrid()

        self.create_subscription(
            OccupancyGrid,
            'map',
            self.mission_callback,
            qos_profile=qos_policy
        )

        self.position_client_ = self.create_client(
            GetPosition,
            'current_position',
        )

        self.recognition_ = self.create_timer(1.0, self.timer_callback)

    def mission_callback(self, msg):
        # save global map
        self.map = msg

    def timer_callback(self):
        # copy map so it doesnt change while beeing looked at
        map = self.map

        # analyze the map
        x, y, resolution = map.info.width, map.info.height, map.info.resolution
        origin = map.info.origin
        print(origin.position.x, origin.position.y)
        # current_position = self.get_location()


    def get_location(self):
        # get current loaction
        get_current_pose = self.position_client_.call_async(GetPosition.Request())
        rclpy.spin_until_future_complete(self, get_current_pose)

        return get_current_pose.result()



# main
def main():
    rclpy.init()
    node = MapRecognition()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()