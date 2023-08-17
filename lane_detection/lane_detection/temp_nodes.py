#!/usr/bin/env python3

import rclpy
import cv2
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class test(Node):

    def __init__(self):
        super().__init__("temp_test")
        self.pub_test_image = self.create_publisher(CompressedImage, '/test_image', 1)
        self.get_logger().info("Publishing test_image has started")
        self.timer_ = self.create_timer(1.0, self.publish_image)
        self.cv_bridge = CvBridge()


    def publish_image(self):
        # get test image
        image = cv2.imread("~/lane_detection/lane_detection/test_lane_image_2.png")

        # Convert the image to a ROS message
        msg = self.cv_bridge.cv2_to_compressed_imgmsg(image)

        # Publish the image
        self.pub_test_image.publish(msg)
        self.get_logger().info("Published image")




def main(args=None):
    rclpy.init(args=args)
    node = test()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=='__main__':
    main()