#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

#from std_msgs.msg import String
from geometry_msgs.msg import Twist
from turtlebot3_interfaces.msg import Drive          # to be created msg, takes to values speed (0,max?) and turn (-max?, max?)

class MotorDriver(Node):
    def __init__(self):
        self.speed_ = 0.0
        self.turn_ = 0.0
        super().__init__("motor_driver")
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscriber_ = self.create_subscription(
            Drive,
            'motor_driver',
            self.listener_callback,
            qos_profile=qos_policy)
        
        self.publisher_ = self.create_publisher(
            Twist,
            'cmd_vel',
            1)
        
        # how often the Twist is sent to the TurtleBot3:
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    # called by subscriber
    def listener_callback(self, msg):
        self.speed_ = msg.speed
        self.turn_ = msg.turn

    # called by publisher
    def timer_callback(self):
        # publish the movement information
        msg = Twist()
        msg.linear.x = self.speed_
        msg.angular.z = self.turn_
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    motor_driver_node = MotorDriver()
    rclpy.spin(motor_driver_node)

    motor_driver_node.destroy_node()
    rclpy.shutdown()