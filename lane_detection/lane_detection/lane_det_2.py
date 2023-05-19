#!/usr/bin python3
# -*- coding: utf-8 -*-

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image




class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__("lane_detection_node_2")
        self.subscription = self.create_subscription(CompressedImage, "/test_image", self.image_callback, 1)
        self.cv_bridge = CvBridge()


    def image_callback(self, msg):
        cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
        cv2.imshow("origianl", cv_image)
        result = self.detect_lanes(cv_image)

        # Display the processed image (for visualization purposes)
        cv2.imshow("Lane_Detection_Result", result)
        cv2.waitKey(1)

        self.get_logger().info("Image recived")


    def detect_lanes(self, image):
        # Convert image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        cv2.imshow("gray", gray)

        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        cv2.imshow("blur", blur)

        # Perform Canny edge detection
        edges = cv2.Canny(blur, 50, 150)
        cv2.imshow("Canny edge detection", edges)

        # Define a region of interest (ROI)
        height, width = edges.shape[:2]
        roi_vertices = [
            (0, height),
            (width / 2, height / 2),
            (width, height)
        ]
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, np.array([roi_vertices], dtype=np.int32), 255)
        roi_edges = cv2.bitwise_and(edges, mask)
        cv2.imshow("roi_edges", roi_edges)

        # Perform Hough line transformation
        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=20,
            minLineLength=20,
            maxLineGap=300
        )

        # Separate lines into left and right lanes
        left_lines, right_lines = [], []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero
            if slope < -0.5:
                left_lines.append(line[0])
            elif slope > 0.5:
                right_lines.append(line[0])

        # Fit a line to each set of lane lines (left and right)
        left_lane = self.fit_lane_line(left_lines, height)
        right_lane = self.fit_lane_line(right_lines, height)

        # Create a blank image to draw the lanes on
        lane_image = np.zeros_like(image)

        # Draw the left and right lanes on the lane image
        self.draw_lane_line(lane_image, left_lane)
        self.draw_lane_line(lane_image, right_lane)

        # Combine the lane image with the original image
        result = cv2.addWeighted(image, 0.8, lane_image, 1, 1)

        return result


    def fit_lane_line(self, lines, height):
        if len(lines) == 0:
            return None

        # Convert lines to a 2D array of points
        points = np.array(lines)

        # Calculate the best-fit line using linear regression
        x = np.reshape(points[:, [0, 2]], (-1, 1))
        y = np.reshape(points[:, [1, 3]], (-1, 1))
        coeffs = np.polyfit(np.squeeze(x), np.squeeze(y), 1)

        # Calculate the start and end points of the lane line
        y1 = height
        y2 = int(height * 0.6)
        x1 = int((y1 - coeffs[1]) / coeffs[0])
        x2 = int((y2 - coeffs[1]) / coeffs[0])

        return ((x1, y1), (x2, y2))


    def draw_lane_line(self, image, lane):
        if lane is not None:
            cv2.line(image, lane[0], lane[1], (0, 255, 0), 5)




def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
