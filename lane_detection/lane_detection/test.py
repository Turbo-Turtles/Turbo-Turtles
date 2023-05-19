#!/usr/bin/env python3

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class ImageSubscriberNode(Node):

    def nothing(self):
        pass


    # Create a window
    cv2.namedWindow('image')

    # create trackbars for coordinat change
    cv2.createTrackbar('p1_x','image',0,322,nothing)
    cv2.createTrackbar('p2_x','image',322,645,nothing)
    cv2.createTrackbar('p12_y','image',0,488,nothing)
    cv2.createTrackbar('p3_x','image',0,322,nothing)
    cv2.createTrackbar('p4_x','image',322,645,nothing)

    # Set default value for coordinates trackbars.
    cv2.setTrackbarPos('p2_x', 'image', 645)
    cv2.setTrackbarPos('p4_x', 'image', 645)

    def __init__(self):
        super().__init__("image_subscriber_node")
        self.subscription = self.create_subscription(Image, "/camera/image_raw", self.image_callback, 1)
        self.cv_bridge = CvBridge()

    
    def image_callback(self, msg):

        # Convert ROS Image message to OpenCV image
        cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")

        # Display the image
        cv2.imshow("Image Subscriber", cv_image)
        self.get_logger().info("Image recived")

        p1_x = cv2.getTrackbarPos('p1_x', 'image')
        p2_x = cv2.getTrackbarPos('p2_x', 'image')
        p12_y = cv2.getTrackbarPos('p12_y', 'image')

        p3_x = cv2.getTrackbarPos('p3_x', 'image')
        p4_x = cv2.getTrackbarPos('p4_x', 'image')

    
        # Locate points of the documents
        # or object which you want to transform
        pts1 = np.float32([[p1_x, p12_y], [p2_x, p12_y],
                           [p3_x, 488], [p4_x, 488]])
        pts2 = np.float32([[0, 0],   [645, 0],
                           [0, 650], [645, 650]])
        
        # Apply Perspective Transform Algorithm
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(cv_image, matrix, (645, 650))
        
        # Wrap the transformed image
        cv2.imshow('image', result) # Transformed Capture
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ImageSubscriberNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
