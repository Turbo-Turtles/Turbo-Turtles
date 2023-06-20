#!/usr/bin python3
# -*- coding: utf-8 -*-

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage, Image

class LaneDetectionNode(Node):

    def nothing(self):
        pass


    # Create a window
    cv2.namedWindow('white_threshold')
    cv2.namedWindow('yellow_threshold')
    cv2.namedWindow('crop_image')

    # create trackbars for color change
    cv2.createTrackbar('Hue_white_min','white_threshold',0,179,nothing) # Hue is from 0-179 for Opencv
    cv2.createTrackbar('Hue_white_max','white_threshold',0,179,nothing)
    cv2.createTrackbar('Sat_white_min','white_threshold',0,255,nothing)
    cv2.createTrackbar('Sat_white_max','white_threshold',0,255,nothing)
    cv2.createTrackbar('Val_white_min','white_threshold',0,255,nothing)
    cv2.createTrackbar('Val_white_max','white_threshold',0,255,nothing)

    cv2.createTrackbar('Hue_yellow_min','yellow_threshold',0,179,nothing)
    cv2.createTrackbar('Hue_yellow_max','yellow_threshold',0,179,nothing)
    cv2.createTrackbar('Sat_yellow_min','yellow_threshold',0,255,nothing)
    cv2.createTrackbar('Sat_yellow_max','yellow_threshold',0,255,nothing)
    cv2.createTrackbar('Val_yellow_min','yellow_threshold',0,255,nothing)
    cv2.createTrackbar('Val_yellow_max','yellow_threshold',0,255,nothing)

    """
    # Set default value for HSV trackbars.
    cv2.setTrackbarPos('Hue_white_min', 'white_threshold', 55)
    cv2.setTrackbarPos('Hue_white_max', 'white_threshold', 110)
    cv2.setTrackbarPos('Sat_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Sat_white_max', 'white_threshold', 40)
    cv2.setTrackbarPos('Val_white_min', 'white_threshold', 210)
    cv2.setTrackbarPos('Val_white_max', 'white_threshold', 255)

    cv2.setTrackbarPos('Hue_yellow_min','yellow_threshold', 30)
    cv2.setTrackbarPos('Hue_yellow_max','yellow_threshold', 46)
    cv2.setTrackbarPos('Sat_yellow_min','yellow_threshold', 40)
    cv2.setTrackbarPos('Sat_yellow_max','yellow_threshold', 255)
    cv2.setTrackbarPos('Val_yellow_min','yellow_threshold', 205)
    cv2.setTrackbarPos('Val_yellow_max','yellow_threshold', 235)
    """

    
    # Set default value for HSV trackbars.  :: Rosbag: Turbo-Turtles_record_05
    cv2.setTrackbarPos('Hue_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Hue_white_max', 'white_threshold', 179)
    cv2.setTrackbarPos('Sat_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Sat_white_max', 'white_threshold', 40)
    cv2.setTrackbarPos('Val_white_min', 'white_threshold', 210)
    cv2.setTrackbarPos('Val_white_max', 'white_threshold', 255)

    cv2.setTrackbarPos('Hue_yellow_min','yellow_threshold', 25)
    cv2.setTrackbarPos('Hue_yellow_max','yellow_threshold', 50)
    cv2.setTrackbarPos('Sat_yellow_min','yellow_threshold', 55)
    cv2.setTrackbarPos('Sat_yellow_max','yellow_threshold', 190)
    cv2.setTrackbarPos('Val_yellow_min','yellow_threshold', 105)
    cv2.setTrackbarPos('Val_yellow_max','yellow_threshold', 225)
    

    """
    # Set default value for HSV trackbars.  :: GAZEBO-Simulation
    cv2.setTrackbarPos('Hue_white_min', 'white_threshold', 10)
    cv2.setTrackbarPos('Hue_white_max', 'white_threshold', 4)
    cv2.setTrackbarPos('Sat_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Sat_white_max', 'white_threshold', 4)
    cv2.setTrackbarPos('Val_white_min', 'white_threshold', 150)
    cv2.setTrackbarPos('Val_white_max', 'white_threshold', 255)

    cv2.setTrackbarPos('Hue_yellow_min','yellow_threshold', 28)
    cv2.setTrackbarPos('Hue_yellow_max','yellow_threshold', 32)
    cv2.setTrackbarPos('Sat_yellow_min','yellow_threshold', 230)
    cv2.setTrackbarPos('Sat_yellow_max','yellow_threshold', 255)
    cv2.setTrackbarPos('Val_yellow_min','yellow_threshold', 150)
    cv2.setTrackbarPos('Val_yellow_max','yellow_threshold', 255)
    """


    '''
    # Set default value for HSV trackbars.  :: Rosbag: Turbo-Turtles_TurboTurtles
    cv2.setTrackbarPos('Hue_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Hue_white_max', 'white_threshold', 179)
    cv2.setTrackbarPos('Sat_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Sat_white_max', 'white_threshold', 20)
    cv2.setTrackbarPos('Val_white_min', 'white_threshold', 220)
    cv2.setTrackbarPos('Val_white_max', 'white_threshold', 255)

    cv2.setTrackbarPos('Hue_yellow_min','yellow_threshold', 30)
    cv2.setTrackbarPos('Hue_yellow_max','yellow_threshold', 40)
    cv2.setTrackbarPos('Sat_yellow_min','yellow_threshold', 130)
    cv2.setTrackbarPos('Sat_yellow_max','yellow_threshold', 255)
    cv2.setTrackbarPos('Val_yellow_min','yellow_threshold', 110)
    cv2.setTrackbarPos('Val_yellow_max','yellow_threshold', 255)
    '''



    # create trackbars for image cropping
    cv2.createTrackbar('p1_x','crop_image',0,100,nothing)
    cv2.createTrackbar('p2_x','crop_image',0,100,nothing)
    cv2.createTrackbar('p12_y','crop_image',0,100,nothing)
    cv2.createTrackbar('p3_x','crop_image',0,100,nothing)
    cv2.createTrackbar('p4_x','crop_image',0,100,nothing)

    # Set default value for image cropping trackbars.
    cv2.setTrackbarPos('p12_y', 'crop_image', 70)
    cv2.setTrackbarPos('p2_x', 'crop_image', 100)
    cv2.setTrackbarPos('p4_x', 'crop_image', 100)



    def __init__(self):
        super().__init__("lane_detection_node_3")
        self.cv_bridge = CvBridge()

        self.sub_image_original = self.create_subscription(CompressedImage, "/image_raw/compressed", self.image_callback, 1) #create image subscriber

        self.pub_vel_cmd = self.create_publisher(Twist, "cmd_vel", 1) #create movement value publisher
        self.pub_image_lane = self.create_publisher(CompressedImage, "/lane_image_out", 1) #create lane image publisher

    
    def image_callback(self, img_msg):
        self.cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")
        #__+__cv2.imshow("origianl", cv_image)
        #__+__self.get_logger().info("Image recived")

        height, width = self.cv_image.shape[:2]
        self.window_width = width
        self.window_height = height
        self.window_size = width * height

        #__+__self.get_logger().info(str(height)+ ","+ str(width))

        croped_img = self.crop_image(self.cv_image)

        result, white_lane, yellow_lane = self.detect_lane(croped_img)

        # Display the processed image (for visualization purposes)
        #__+__cv2.imshow("Lane_Detection_Result", result)
        cv2.waitKey(1)

        self.driving(white_lane, yellow_lane, result)
    
        '''# Process lane detection data and calculate desired steering angle and velocity
        if white_lane is not None and yellow_lane is not None:
            # Calculate the center lane coordinates
            w1, w2 = white_lane
            y1, y2 = yellow_lane

            center_lane_x_1 = int((w1[0] + y1[0]) / 2)
            center_lane_x_2 = int((w2[0] + y2[0]) / 2)
            center_lane_y = int(w2[1] + y2[1])

            # Draw the center lane on the image
            cv2.line(result, (center_lane_x_2, center_lane_y), (center_lane_x_1, self.window_height), (0, 0, 255), 5)

            # Display the image with the lanes
            self.pub_image_lane.publish(self.cv_bridge.cv2_to_compressed_imgmsg(result, "jpg"))
            cv2.imshow("Lane Detection driving line", result)
            cv2.waitKey(1)
        else:
            self.pub_image_lane.publish(self.cv_bridge.cv2_to_compressed_imgmsg(result, "jpg"))
            cv2.imshow("Lane Detection driving line", result)
            cv2.waitKey(1)'''



    def crop_image(self, image):
        p1_x = cv2.getTrackbarPos('p1_x', 'crop_image')
        p2_x = cv2.getTrackbarPos('p2_x', 'crop_image')
        p12_y = cv2.getTrackbarPos('p12_y', 'crop_image')

        p3_x = cv2.getTrackbarPos('p3_x', 'crop_image')
        p4_x = cv2.getTrackbarPos('p4_x', 'crop_image')

    
        # Locate points of the documents
        # or object which you want to transform
        pts1 = np.float32([[((self.window_width * p1_x)/100), ((self.window_height * p12_y)/100)], [((self.window_width * p2_x)/100), ((self.window_height * p12_y)/100)],
                           [((self.window_width * p3_x)/100),   self.window_height],               [((self.window_width * p4_x)/100),   self.window_height]])
        pts2 = np.float32([[0, 0],   [self.window_width, 0],
                           [0, self.window_height], [self.window_width, self.window_height]])
        
        # Apply Perspective Transform Algorithm
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(image, matrix, (self.window_width, self.window_height))
        
        # Wrap the transformed image
        cv2.imshow('crop_image', result) # Transformed Capture

        return result

    
    def detect_lane(self, image):
        # find White and Yellow Lines
        white_line = self.maskWhiteLine(image)
        yellow_line = self.maskYellowLine(image)

        # Fit a line to each set of lane lines (left and right)
        white_lane_line = self.fit_lane_line(white_line)
        yellow_lane_line = self.fit_lane_line(yellow_line)

        # Create a blank image to draw the lanes on
        lane_image = np.zeros_like(image)

        # Draw the left and right lanes on the lane image
        self.draw_lane_line(lane_image, white_lane_line)
        self.draw_lane_line(lane_image, yellow_lane_line)

        # Combine the lane image with the original image
        result = cv2.addWeighted(image, 0.7, lane_image, 1, 1)

        return result, white_lane_line, yellow_lane_line
        #pass


    def maskWhiteLine(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_min = cv2.getTrackbarPos('Hue_white_min', 'white_threshold')
        Hue_max = cv2.getTrackbarPos('Hue_white_max', 'white_threshold')
        Saturation_min = cv2.getTrackbarPos('Sat_white_min', 'white_threshold')
        Saturation_max = cv2.getTrackbarPos('Sat_white_max', 'white_threshold')
        Value_min = cv2.getTrackbarPos('Val_white_min', 'white_threshold')
        Value_max = cv2.getTrackbarPos('Val_white_max', 'white_threshold')

        # define range of white color in HSV
        lower_white = np.array([Hue_min, Saturation_min, Value_min])
        upper_white = np.array([Hue_max, Saturation_max, Value_max])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Bitwise-AND mask and original image
        lane = cv2.bitwise_and(image, image, mask = mask)
        cv2.imshow("white_threshold", lane)

        # Convert image to grayscale
        gray = cv2.cvtColor(lane, cv2.COLOR_BGR2GRAY)
        #__+__cv2.imshow("gray_w", gray)

        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (13, 13), 2) #standard: (15, 15), 0
        #__+__cv2.imshow("blur_w", blur)

        # Perform Canny edge detection
        edges = cv2.Canny(blur, 75, 200) #standard: 50, 150
        #__+__cv2.imshow("Canny edge detection_w", edges)

        # Define a region of interest (ROI)
        roi_vertices = [
            (self.window_width * 0.2, 0),
            #(self.window_width * 0.9, 0),
            (self.window_width * 1.0, 0),
            (self.window_width, self.window_height),
            (self.window_width * 0.8, self.window_height)
        ]
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, np.array([roi_vertices], dtype=np.int32), 255)
        roi_edges = cv2.bitwise_and(edges, mask)
        cv2.imshow("roi_edges_w", roi_edges)

        # Perform Hough line transformation
        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,   #standard: 20
            minLineLength=50, #standard: 20
            maxLineGap=100   #standard: 300
        )

        w_lines = []
        try:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero
                if slope > -0.5:         # maybe change parameter
                    w_lines.append(line[0])
        except:
            w_lines = []
        
        '''w_lines = []
        try:
            for line in lines:
                    w_lines.append(line[0])
        except:
            w_lines = []'''

        return w_lines
    

    def maskYellowLine(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_min = cv2.getTrackbarPos('Hue_yellow_min', 'yellow_threshold')
        Hue_max = cv2.getTrackbarPos('Hue_yellow_max', 'yellow_threshold')
        Saturation_min = cv2.getTrackbarPos('Sat_yellow_min', 'yellow_threshold')
        Saturation_max = cv2.getTrackbarPos('Sat_yellow_max', 'yellow_threshold')
        Value_min = cv2.getTrackbarPos('Val_yellow_min', 'yellow_threshold')
        Value_max = cv2.getTrackbarPos('Val_yellow_max', 'yellow_threshold')

        # define range of white color in HSV
        lower_yellow = np.array([Hue_min, Saturation_min, Value_min])
        upper_yellow = np.array([Hue_max, Saturation_max, Value_max])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        lane = cv2.bitwise_and(image, image, mask = mask)
        cv2.imshow("yellow_threshold", lane)

        # Convert image to grayscale
        gray = cv2.cvtColor(lane, cv2.COLOR_BGR2GRAY)
        #__+__cv2.imshow("gray_y", gray)

        # Apply Gaussian blur
        blur = cv2.GaussianBlur(gray, (13, 13), 1) #standard: (15, 15), 0
        #__+__cv2.imshow("blur_y", blur)

        # Perform Canny edge detection
        edges = cv2.Canny(blur, 75, 200) #standard: 50, 150
        #__+__cv2.imshow("Canny edge detection_y", edges)

        # Define a region of interest (ROI)
        roi_vertices = [
            #(self.window_width * 0.1, 0),
            (self.window_width * 0, 0),
            (self.window_width * 0.8, 0),
            (self.window_width * 0.2, self.window_height),
            (0, self.window_height)
        ]
        mask = np.zeros_like(edges)
        cv2.fillPoly(mask, np.array([roi_vertices], dtype=np.int32), 255)
        roi_edges = cv2.bitwise_and(edges, mask)
        cv2.imshow("roi_edges_y", roi_edges)

        # Perform Hough line transformation
        lines = cv2.HoughLinesP(
            roi_edges,
            rho=1,
            theta=np.pi / 180,
            threshold=50,   #standard: 20
            minLineLength=50, #standard: 20
            maxLineGap=100  #standard: 300
        )

        y_lines = []
        try:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero
                if slope < 0.5:         # maybe change parameter
                    y_lines.append(line[0])
        except:
            y_lines = []
        
        '''y_lines = []
        try:
            for line in lines:
                y_lines.append(line[0])
        except:
            y_lines = []'''

        return y_lines


    def fit_lane_line(self, lines):
        if len(lines) == 0:
            return None

        # Convert lines to a 2D array of points
        points = np.array(lines)

        # Calculate the best-fit line using linear regression
        x = np.reshape(points[:, [0, 2]], (-1, 1))
        y = np.reshape(points[:, [1, 3]], (-1, 1))
        coeffs = np.polyfit(np.squeeze(x), np.squeeze(y), 1)

        # Calculate the start and end points of the lane line
        y1 = self.window_height
        y2 = int(self.window_height * 0.3)
        x1 = int((y1 - coeffs[1]) / coeffs[0])
        x2 = int((y2 - coeffs[1]) / coeffs[0])

        return ((x1, y1), (x2, y2))
    

    def draw_lane_line(self, image, lane):
        if lane is not None:
            cv2.line(image, lane[0], lane[1], (255, 150, 0), 10)

    
    def driving(self, white_lane, yellow_lane, img):
        # Define control parameters
        kp = 0.02  # Proportional gain
        max_vel = 0.1  # Constant velocity
        velocity = max_vel
        steering_angle = 0.0
        self.old_deviation = 0
        self.first = True

        if white_lane is not None and yellow_lane is not None:
            # Calculate the center of the lane
            #__+__lane_center = ((white_lane[0][0] + yellow_lane[0][0]) // 2, (white_lane[1][1] + yellow_lane[1][1]) // 2)

            w1, w2 = white_lane
            y1, y2 = yellow_lane

            center_lane_x_1 = int((w1[0] + y1[0]) / 2)
            center_lane_x_2 = int((w2[0] + y2[0]) / 2)
            center_lane_y = int(w2[1] + y2[1])

            lane_center = (center_lane_x_2, center_lane_y)
            center_lane = ((center_lane_x_2, center_lane_y),(center_lane_x_1, self.window_height))

            # Calculate the deviation from the center of the image
            image_center = self.window_width / 2
            deviation = lane_center[0] - image_center

            # Draw the center lane on the image
            #__+__cv2.line(img, lane_center, (int(self.window_width/2), self.window_height), (0, 0, 255), 5)
            cv2.line(img, center_lane[0], center_lane[1], (0, 0, 255), 5)

            if self.first == True:
                self.old_deviation = deviation
                self.first = False

            if abs(self.old_deviation - deviation) > 50:
                deviation = self.old_deviation

            # Calculate the desired steering angle based on the deviation and control parameters
            if deviation < 100 and deviation > -100:
                steering_angle = kp * -deviation
                velocity = max_vel
            elif deviation > 100:
                steering_angle = -1.0
                velocity = max_vel/(deviation*kp*2)
            elif deviation < -100:
                steering_angle = 1.0
                velocity = -max_vel/(deviation*kp*2)

        elif white_lane is None:
            steering_angle = -0.2
            velocity = max_vel/5

        elif yellow_lane is None:
            steering_angle = 0.2
            velocity = max_vel/5

        else:
            steering_angle = 0.2
            velocity = 0

        # Display the image with the lanes
        self.pub_image_lane.publish(self.cv_bridge.cv2_to_compressed_imgmsg(img, "jpg"))
        cv2.imshow("Lane Detection driving line", img)
        cv2.waitKey(1)
        
        self.get_logger().info('steer.: ' + str(steering_angle) + '; vel.: ' + str(velocity))

        # Create Twist message with desired linear and angular velocities
        twist_msg = Twist()
        twist_msg.linear.x = velocity
        twist_msg.angular.z = steering_angle

        # Publish the twist message to control the TurtleBot3
        self.pub_vel_cmd.publish(twist_msg)
    


    
def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()