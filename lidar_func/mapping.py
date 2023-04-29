#!/usr/bin/env python3

# ros
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.action import ActionClient
from rcl_interfaces.msg import ParameterType
from action_msgs.msg import GoalStatus

# messages
from std_msgs.msg import Float32
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav2_msgs.action import NavigateToPose

from map_msgs.msg import ProjectedMap

from map_msgs.srv import GetPointMap
from nav2_msgs.srv import GetCostmap
from turtlebot3_example.turtlebot3_obstacle_detection import main as lol

from slam_toolbox.srv import SaveMap


class Mapper(Node):
    def __init__(self):
        self.count = 0
        super().__init__("Mapper")
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.map_sub_ = self.create_subscription(
            ProjectedMap,
            'map',
            self.listener_callback,
            qos_profile=qos_policy)
        
        self.is_alive_ = self.create_timer(1, self.timer_callback)

    def listener_callback(self, msg):
        print(type(msg))
        self.count += 1

    def timer_callback(self):
        print("alive", self.count)





def main(args=None):
    rclpy.init(args=args)
    mapper = Mapper()
    rclpy.spin(mapper)
    mapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()