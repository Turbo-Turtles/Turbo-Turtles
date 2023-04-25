#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from math import sin, cos, radians

#from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LidarSubscriberNode(Node):
    def __init__(self):
        self.distMin_ = 100
        self.distMinCount_ = 0
        self.distMinDeg_ = 0
        self.next_distLeft_ = 0
        self.step_ = 0.03
        super().__init__("lidar_subscriber")
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            qos_profile=qos_policy)
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def listener_callback(self, msg):
        self.distMin_ = 100
        self.counter_ = 0
        #local_ranges = msg.ranges.tolist()
        for item in msg.ranges:
            if (item != 0.0 and item <= self.distMin_):
                self.distMin_ = item
                self.distMinDeg_ = self.counter_
                #print(item, self.counter_)

                if (self.counter_ < 90):
                    x = round(sin(radians(self.counter_)) * self.distMin_, 2)
                    y = round(cos(radians(self.counter_)) * self.distMin_, 2)
                    #print('x: {0}, y: {1}'.format(x, y))

                    if (y == self.step_):
                        self.next_distLeft_ = x

            self.counter_ += 1
        
        self.distMinCount_ = msg.ranges.count(self.distMin_)

        #self.get_logger().info('Count: "%s"' % self.distMinCount_)
        self.get_logger().info('Minimal: "{0}" Degree: "{1}" Times: "{2}"'.format(self.distMin_, self.distMinDeg_, self.distMinCount_))
        self.get_logger().info('Next distance on the left side: {0}'.format(self.next_distLeft_))

    # Vorn am Turtlebot ist 0°
    # Von da nach links aufsteigend bis wieder 360°
    # vorn: 0 / 360
    # links: 90
    # hinten: 180
    # rechts: 270

    def timer_callback(self):
        if (self.distMin_ > 0.25):
            if (self.distMinDeg_ <= 180 and self.distMinDeg_ > 10):
                speed = 0.0
                turn = 0.4 # turn left
            
            elif (self.distMinDeg_ < 350 and self.distMinDeg_ > 180):
                speed = 0.0
                turn = -0.4 # turn right
            
            else:
                speed = 0.1 # drive straight
                turn = 0.0

        elif (self.distMin_ <= 0.25):
            if (self.distMinDeg_ < 80 or self.distMinDeg_ > 270):
                speed = 0.0
                turn = -0.4 # turn right

            elif (self.distMinDeg_ < 270 and self.distMinDeg_ > 100):
                speed = 0.0
                turn = 0.4 # turn left

            else:
                if (self.next_distLeft_ <= 0.25):
                    speed = 0.1 # drive straight
                    turn = 0.0

                else:
                    speed = 0.0
                    turn = 0.4 # turn left

        else:
            speed = 0.0 # stop
            turn = 0.0
            print('stop')


        # publish the movement information
        msg = Twist()
        msg.linear.x = speed
        msg.angular.z = turn
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriberNode()
    rclpy.spin(lidar_subscriber)

    lidar_subscriber.destroy_node()
    rclpy.shutdown()