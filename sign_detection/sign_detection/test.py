#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

class SignDetection(Node):
    def __init__(self):
        super().__init__("sign_detection")
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        
        self.subscriber_ = self.create_subscription(
            CompressedImage,
            "/image_raw/compressed",
            self.listener_callback,
            qos_profile=qos_policy)
        
        timer_period = 1

        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.bridge = CvBridge()

        #self.sign_text = np.zeros((100, 100, 3), np.uint8)

        self.pub_img = CompressedImage()

        self.publisher_ = self.create_publisher(
            CompressedImage,
            "/test_detection/compressed",
            1)
        
    def listener_callback(self, img_msg):
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        #ros_msg = self.bridge.cv2_to_imgmsg(cv_image)
        #ros_msg.header = msg.header
        #img = self.bridge.imgmsg_to_cv2(ros_msg)

        
        
        sign = cv2.CascadeClassifier('/home/marvin/ros2_ws/Turbo-Turtles/sign_detection/sign_detection/stop_sign.xml')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        sign_scaled = sign.detectMultiScale(image=gray, scaleFactor=1.3, minNeighbors=5)
        # sign_text = cv_image

        for (x, y, w, h) in sign_scaled:
            sign_rectangle = cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 3)
            sign_text = cv2.putText(sign_rectangle,
                                    "Sign",
                                    (x, y+h+30),
                                    cv2.FONT_HERSHEY_SIMPLEX,
                                    1,
                                    (0, 0, 255),
                                    2,
                                    cv2.LINE_4)
        
        # cv2.imshow("img", cv_image)
        # cv2.waitKey(1)

        # try:            
        #     self.pub_img = self.bridge.cv2_to_compressed_imgmsg(cv_image, "jpg")    
        # except CvBridgeError as e:
        #     print(e)
        
        self.publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image))
        # self.publisher_.publish(self.bridge.cv2_to_compressed_imgmsg(cv_image))
        # self.get_logger().info('image send')
            
    
    def timer_callback(self):
        #self.get_logger().info('Test')
        pass
        

def main(args=None):
    rclpy.init(args=args)
    sign_detection_node = SignDetection()
    rclpy.spin(sign_detection_node)

    sign_detection_node.destroy_node()
    rclpy.shutdown()
