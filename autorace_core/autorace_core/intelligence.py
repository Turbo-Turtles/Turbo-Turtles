#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlebot3_interfaces.msg import Mission, Progress, Sign

class Intelligence(Node):
    def __init__(self):
        super().__init__("intelligence_node")

        self.sections = ["traffic", "intersection", "construction", "parking", "crossing", "tunnel"]

        self.active_section = self.sections[0]
        self.section_changed = False
        self.lane_following_active = False

        self.running = True

        # qos policy
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        # subscriber
        self.sub_progress = self.create_subscription(
            Progress,
            'progress',
            self.progress_callback,
            qos_profile=qos_policy
        )

        self.sub_sign = self.create_subscription(
            Sign,
            'sign',
            self.sign_callback,
            qos_profile=qos_policy
        )

        # publisher
        self.pub_mission = self.create_publisher(
            Mission,
            'mission',
            1
        )

        # timer
        self.updater = self.create_timer(
            0.02,
            self.updater_callback
        )

    # subscriber callbacks
    def progress_callback(self, msg):
        # if active section traffic
        # if error from lane following : stop execution

        # if active section intersection
        # if error from lane following : stop execution

        # if active section construction
        # if error from lane following : stop execution
        # if error from construction mission : stop execution

        # if active section parking
        # if error from lane following : stop execution

        # if active section crossing
        # if error from lane following : stop execution

        # if active section tunnel
        # if error from lane following : stop execution
        # if error from construction mission : stop execution

        return
    
    def sign_callback(self, msg):
        # if active section traffic
        # if traffic light red / orange : no lane following
        # if traffic light green : lane following

        # if active section traffic
        # if intersection sign : set actve section intersection

        # if active section intersection
        # if arrow left : left to lane following
        # if arrow right : right to lane following

        # if active section intersection
        # if construction sign : set active section construction, no lane following, go for construction mission

        # if active section construction
        # if parking sign : set active section parking, left to lane following

        # if active section parking
        # if arrow left : set active section crossing, left to lane following

        # if active section crossing
        # if crossing closed : no lane following
        # if crossing open : lane following
        # set active mission tunnel

        # if active mission tunnel
        # if tunnel sign : go for tunnel mission

        # if active mission tunnel
        # if traffic light : no lane following, set self.running to False

        return
    
    # timer callbacks
    def updater_callback(self):
        # publish the current mission, if the section changed
        if self.section_changed:
            msg = Mission()
            msg.mission_name = self.active_section
            self.pub_mission(msg)

        # publish state of lane following
        msg = Mission()
        msg.mission_name = "lane"
        msg.state = self.lane_following_active
        self.pub_mission(msg)



# main
def main():
    rclpy.init()
    node = Intelligence()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()