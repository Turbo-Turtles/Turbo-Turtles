#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from turtlebot3_interfaces.msg import Mission, Progress, Sign

class Intelligence(Node):
    def __init__(self):
        super().__init__("intelligence_node")

        self.sections = ["traffic", "intersection", "construction", "parking", "crossing", "tunnel"]
        self.turns = ["left", "right"]

        self.active_section = 0
        self.section_changed = False
        self.lane_following_active = False
        self.lane_changed = False
        self.turn = 0

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

        # self.pub_lane = self.create_publisher(
        #     Mission,
        #     'lane_state',
        #     1
        # )

        # timer
        self.updater = self.create_timer(
            0.02,
            self.updater_callback
        )

    # subscriber callbacks
    def progress_callback(self, msg):
        # basically just a fail save, so the robot stop, when any node signals an error
        if msg.state == False:
            self.running = False

        # if a nav2 task (construction and tunnel) is done, it sends a message with True
        # this enables the lane following again
        elif msg.sender == "nav2":
            self.lane_following_active = True
        
        # if a turn is completed
        elif msg.sender == "turn":
            self.lane_following_active = True
    
    def sign_callback(self, msg):
        # if active section traffic
        # if traffic light red / orange : no lane following
        # if traffic light green : lane following

        # if active section traffic
        # if intersection sign : set actve section intersection
        if self.active_section == 0:
            if msg.sign == "traffic":
                if msg.state == 0 or msg.state == 1:
                    self.lane_following_active = False
                elif msg.state == 2:
                    self.lane_following_active = True
            
            if msg.sign == "intersection":
                self.active_section += 1
                self.section_changed = True

        # if active section intersection
        # if arrow left : left to lane following
        # if arrow right : right to lane following

        # if active section intersection
        # if construction sign : set active section construction, no lane following, go for construction mission
        if self.active_section == 1:
            if msg.sign == "left":
                self.turn = 2
                self.lane_following_active = False
            elif msg.sign == "right":
                self.turn = 1
                self.lane_following_active = False

            if msg.sign == "contruction":
                self.active_section += 1
                self.section_changed = True

                self.lane_following_active = False

        # if active section construction
        # if parking sign : set active section parking, left to lane following
        if self.active_section == 2:
            if msg.sign == "parking":
                self.active_section += 1
                self.section_changed = True

                self.turn = 2

        # if active section parking
        # if arrow left : set active section crossing, left to lane following
        if self.active_section == 3:
            if msg.sign == "left":
                self.active_section += 1

                self.turn = 2

        # if active section crossing
        # if crossing closed : no lane following
        # if crossing open : lane following
        # set active mission tunnel
        if self.active_section == 4:
            if msg.sign == "crossing":
                self.section_changed = True

                if msg.state == 0:
                    self.lane_following_active = False
                elif msg.state == 1:
                    self.lane_following_active = True

                    self.active_section += 1

        # if active mission tunnel
        # if tunnel sign : go for tunnel mission

        # if active mission tunnel
        # if traffic light : set self.running to False
        if self.active_section == 5:
            if msg.sign == "tunnel":
                self.section_changed = True

            if msg.sign == "traffic":
                self.running = False
    
    # timer callbacks
    def updater_callback(self):
        if self.running:
            # publish the current mission, if the section changed
            if self.section_changed:
                self.section_changed = False

                msg = Mission()
                msg.mission_name = self.sections[self.active_section]
                msg.complete = True
                self.pub_mission(msg)

            # publish state of lane following
            if self.lane_changed != self.lane_following_active:
                self.lane_changed = self.lane_following_active

                msg = Mission()
                msg.mission_name = "lane"
                msg.complete = self.lane_following_active
                self.pub_mission(msg)

            # signal an upcoming turn
            if self.turn != 0:
                self.turn = 0

                msg = Mission()
                msg.mission_name = self.turns[-self.turn]
                msg.complete = True
                self.pub_mission(msg)

        else:
            # end lane following
            msg = Mission()
            msg.mission_name = "lane"
            msg.complete = False
            self.pub_mission(msg)

            # stop execution
            raise SystemExit



# main
def main():
    rclpy.init()
    node = Intelligence()
    try:
        rclpy.spin(node)
    except SystemExit:
        rclpy.logging.get_logger("Quitting").info('Done')
    node.destroy_node()
    rclpy.shutdown()

    exit(0)


if __name__ == '__main__':
    main()