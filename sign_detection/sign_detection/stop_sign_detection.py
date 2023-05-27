#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class SignDetection(Node):
    def __init__(self):
        super().__init__("sign_detection")
        
        self.subscriber_ = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.listener_callback,
            1)
        
        timer_period = 0.05

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bridge = CvBridge()

        self.sign_text = np.zeros((100, 100, 3), np.uint8)

        self.publisher_ = self.create_publisher(
            CompressedImage,
            "sign_detection",
            1)
    
    def listener_callback(self, msg):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg)
        
        sign = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/sign_detection/stop_sign.xml')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        sign_scaled = sign.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)

        for (x, y, w, h) in sign_scaled:
            sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            self.sign_text = cv2.putText(sign_rectangle,
                                    "Sign",
                                    (x, y+h+30),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1,
                                    (0, 0, 255),
                                    2,
                                    cv2.LINE_4)
    
    def timer_callback(self):
        self.publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(self.sign_text))

def main(args=None):
    rclpy.init(args=args)
    sign_detection_node = SignDetection()
    rclpy.spin(sign_detection_node)

    sign_detection_node.destroy_node()
    rclpy.shutdown()
