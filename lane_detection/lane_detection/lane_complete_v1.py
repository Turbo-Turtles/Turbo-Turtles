#!/usr/bin python3
# -*- coding: utf-8 -*-

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import CompressedImage
from math import acos
from lidar_navigation.free_lot import GetFreeSlot
from turtlebot3_interfaces.msg import Progress, Mission
from lidar_navigation.position_listener import PositionListener

class LaneDetectionNode(Node):

    def nothing(self):
        pass


    # Create a window
    cv2.namedWindow('white_threshold')
    cv2.namedWindow('yellow_threshold')
    cv2.namedWindow('crop_image')
    cv2.namedWindow('parking_line')

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
    
    # Set default value for HSV trackbars.  :: Rosbag: Turbo-Turtles_record_05
    cv2.setTrackbarPos('Hue_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Hue_white_max', 'white_threshold', 179)
    cv2.setTrackbarPos('Sat_white_min', 'white_threshold', 0)
    cv2.setTrackbarPos('Sat_white_max', 'white_threshold', 120)
    cv2.setTrackbarPos('Val_white_min', 'white_threshold', 180)
    cv2.setTrackbarPos('Val_white_max', 'white_threshold', 255)

    cv2.setTrackbarPos('Hue_yellow_min','yellow_threshold', 25)
    cv2.setTrackbarPos('Hue_yellow_max','yellow_threshold', 50)
    cv2.setTrackbarPos('Sat_yellow_min','yellow_threshold', 55)
    cv2.setTrackbarPos('Sat_yellow_max','yellow_threshold', 220)
    cv2.setTrackbarPos('Val_yellow_min','yellow_threshold', 90)
    cv2.setTrackbarPos('Val_yellow_max','yellow_threshold', 240)
    
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
    cv2.setTrackbarPos('p12_y', 'crop_image', 85)
    cv2.setTrackbarPos('p2_x', 'crop_image', 100)
    cv2.setTrackbarPos('p4_x', 'crop_image', 100)

    
    # create trackbars for parking_line image
    cv2.createTrackbar('p1_y','parking_line',0,100,nothing)
    cv2.createTrackbar('p2_y','parking_line',0,100,nothing)

    # Set default value for parking_line image trackbars.
    cv2.setTrackbarPos('p1_y', 'parking_line', 20)
    cv2.setTrackbarPos('p2_y', 'parking_line', 80)



    def __init__(self):
        super().__init__("lane_detection_node_3")
        self.cv_bridge = CvBridge()

        self.sub_image_original = self.create_subscription(CompressedImage, "/image_raw/compressed", self.image_callback, 1) #create image subscriber
        self.sub_mission_state = self.create_subscription(Mission, '/mission', self.mission_state, 1) #

        self.pub_vel_cmd = self.create_publisher(Twist, "cmd_vel", 1) #create movement value publisher
        self.pub_image_lane = self.create_publisher(CompressedImage, "/lane_image_out", 1) #create lane image publisher
        self.pub_progress = self.create_publisher(Progress, '/mission_progress', 1)

        self.state = 1
        self.counter = 0
        self.counter_set = 10
        self.parking_state = 0
        self.old_deviation = 0
        self.first = True
        self.parking_done = False

    
    def mission_state(self, msg):
        self.state = msg.mission_name

    
    def image_callback(self, img_msg):
        self.cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")

        height, width = self.cv_image.shape[:2]
        self.window_width = width
        self.window_height = height
        self.window_size = width * height

        progress_msg = Progress()


        # croping image for RIO
        croped_img = self.crop_image(self.cv_image)

        match self.state:
            case 0:
                pass
            case 1:
                self.lane_following(croped_img)
            case 2:
                self.right_turn()

                self.state = 0
                progress_msg.state = True
                progress_msg.sender = 'turn'
                self.pub_progress.publish(progress_msg)
            case 3:
                self.left_turn()
                
                self.state = 0
                progress_msg.state = True
                progress_msg.sender = 'turn'
                self.pub_progress.publish(progress_msg)
            case 4:self.state = None
        self.counter = 0
        self.counter_set = 10
        self.parking_state = 0
        self.old_deviation = 0
        self.first = True
        self.parking_done = False

    
    def mission_state(self, msg):
        self.state = msg.mission_name


    def image_callback(self, img_msg):
        self.cv_image = self.cv_bridge.compressed_imgmsg_to_cv2(img_msg, "bgr8")

        height, width = self.cv_image.shape[:2]
        self.window_width = width
        self.window_height = height
        self.window_size = width * height

        '''progress_msg = Progress()'''


        # croping image for RIO
        croped_img = self.crop_image(self.cv_image)

        match self.state:
            case 'lane':
                self.lane_following(croped_img)
            case 'right':
                self.right_turn()

                self.state = 0
                progress_msg.state = True
                progress_msg.sender = 'turn'
                self.pub_progress.publish(progress_msg)
            case 'left':
                self.left_turn()
                
                self.state = 0
                progress_msg.state = True
                progress_msg.sender = 'turn'
                self.pub_progress.publish(progress_msg)
            case 'parking':
                self.parking(croped_img)
                if self.parking_done == True:
                    self.state = 0
                    self.parking_done = False
            case _:
                pass

        cv2.waitKey(1)
                self.parking(croped_img)
                if self.parking_done == True:
                    self.state = 0
                    self.parking_done = False

        cv2.waitKey(1)
        
        cv2.moveWindow('crop_image', 0, 0)
        cv2.moveWindow('yellow_threshold', 450, 0)
        cv2.moveWindow('white_threshold', 900, 0)
        cv2.moveWindow('Lane Detection driving line', 0, 550)
        cv2.moveWindow('roi_edges_y', 450, 550)
        cv2.moveWindow('roi_edges_w', 900, 550)


    def lane_following(self, image):
        # find White and Yellow Lines
        white_line = self.maskWhiteLine(image)
        yellow_line = self.maskYellowLine(image)

        # Fit a line to each set of lane lines (left and right)
        white_lane_line = self.fit_lane_line(white_line)
        yellow_lane_line = self.fit_lane_line(yellow_line)

        # Create a blank image to draw the lanes on
        lane_image = np.zeros_like(image)

        # Draw the left and right lanes on the lane image
        lane_image = self.draw_lane_line(lane_image, white_lane_line)
        lane_image = self.draw_lane_line(lane_image, yellow_lane_line)

        # Combine the lane image with the original image
        result = cv2.addWeighted(image, 0.7, lane_image, 1, 1)

        #__+__cv2.waitKey(1)

        self.driving(white_lane_line, yellow_lane_line, result)


    def parking(self, image):
        img_parking_line = self.crop_parking_line(self.cv_image)

        match self.parking_state:
            case 0:
                self.left_turn()
                self.parking_state = 1
            case 1:
                # find White and Yellow Lines
                yellow_line_R = self.maskYellowLine_2(image)
                yellow_line_L = self.maskYellowLine(image)
                parking_line = self.maskWhite_Parking_Line(img_parking_line)
                

                # Fit a line to each set of lane lines (left and right)
                yellow_lane_line_R = self.fit_lane_line(yellow_line_R)
                yellow_lane_line_L = self.fit_lane_line(yellow_line_L)

                # Create a blank image to draw the lanes on
                lane_image = np.zeros_like(image)

                # Draw the left and right lanes on the lane image
                lane_image = self.draw_lane_line(lane_image, yellow_lane_line_R)
                lane_image = self.draw_lane_line(lane_image, yellow_lane_line_L)

                # Combine the lane image with the original image
                result = cv2.addWeighted(image, 0.7, lane_image, 1, 1)

                self.driving(yellow_lane_line_R, yellow_lane_line_L, result)

                if parking_line is not None:
                    self.parking_state = 2
            case 2:
                self.spot_info = None
                self.spot_info = self.get_lot() # get spot_info
                if self.spot_info is not None:
                    self.parking_state = 3
            case 3:
                if self.spot_info == 'left':
                    self.hard_left_turn()
                else:
                    self.hard_right_turn()
                self.parking_state = 4
            case 4:
                # Create Twist message with desired linear and angular velocities
                twist_msg = Twist()
                twist_msg.linear.x = 0.1
                twist_msg.angular.z = 0.0

                # Publish the twist message to control the TurtleBot3
                self.pub_vel_cmd.publish(twist_msg)
                self.counter =+ 1
                if self.counter > self.counter_set:
                    self.parking_state = 5
                    self.counter = 0
            case 5:
                self.counter =+ 1
                if self.counter > self.counter_set:
                    self.parking_state = 6
                    self.counter = 0
            case 6:
                # Create Twist message with desired linear and angular velocities
                twist_msg = Twist()
                twist_msg.linear.x = -0.1
                twist_msg.angular.z = 0.0

                # Publish the twist message to control the TurtleBot3
                self.pub_vel_cmd.publish(twist_msg)
                self.counter =+ 1
                if self.counter > self.counter_set:
                    self.parking_state = 7
                    self.counter = 0
            case 7:
                if self.spot_info == 'left':
                    self.hard_right_turn()
                else:
                    self.hard_left_turn()
                self.parking_state = 8
            case 8:
                # find White and Yellow Lines
                yellow_line_R = self.maskYellowLine_2(image)
                yellow_line_L = self.maskYellowLine(image)
                parking_line = self.maskWhite_Parking_Line(image)
                

                # Fit a line to each set of lane lines (left and right)
                yellow_lane_line_R = self.fit_lane_line(yellow_line_R)
                yellow_lane_line_L = self.fit_lane_line(yellow_line_L)

                # Create a blank image to draw the lanes on
                lane_image = np.zeros_like(image)

                # Draw the left and right lanes on the lane image
                lane_image = self.draw_lane_line(lane_image, yellow_lane_line_R)
                lane_image = self.draw_lane_line(lane_image, yellow_lane_line_L)

                # Combine the lane image with the original image
                result = cv2.addWeighted(image, 0.7, lane_image, 1, 1)

                self.driving(yellow_lane_line_R, yellow_lane_line_L, result)

                if parking_line is not None:
                    self.parking_state = 9
            case 9:
                self.left_turn()
                self.parking_state = 10
            case 10:
                self.parking_done = True


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
    

    def crop_parking_line(self, image):
        p1_y = cv2.getTrackbarPos('p1_y', 'parking_line')
        p2_y = cv2.getTrackbarPos('p2_y', 'parking_line')

        # Locate points of the documents
        # or object which you want to transform
        pts1 = np.float32([[0, ((self.window_height * p1_y)/100)], [self.window_width, ((self.window_height * p1_y)/100)],
                           [0, ((self.window_height * p2_y)/100)], [self.window_width, ((self.window_height * p2_y)/100)]])
        pts2 = np.float32([[0, 0],   [self.window_width, 0],
                           [0, self.window_height], [self.window_width, self.window_height]])
        
        # Apply Perspective Transform Algorithm
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(image, matrix, (self.window_width, self.window_height))
        
        # Wrap the transformed image
        cv2.imshow('parking_line', result) # Transformed Capture

        return result


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

        return y_lines
    

    def maskYellowLine_2(self, image):
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

        y_lines = []
        try:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                slope = (y2 - y1) / (x2 - x1 + 1e-6)  # Avoid division by zero
                if slope > -0.5:         # maybe change parameter
                    y_lines.append(line[0])
        except:
            y_lines = []

        return y_lines
    

    def maskWhite_Parking_Line(self, image):
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
                if slope > -0.2 and slope < 0.2:         # maybe change parameter
                    w_lines.append(line[0])
        except:
            w_lines = []

        return w_lines


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
        y1 = 1
        y2 = 1 
        x1 = 1
        x2 = 1
        
        y1 = int(self.window_height)
        y2 = int(self.window_height * 0.3)
        x1 = int((y1 - coeffs[1]) / coeffs[0])
        x2 = int((y2 - coeffs[1]) / coeffs[0])

        return ((x1, y1), (x2, y2))
    

    def draw_lane_line(self, image, lane):
        if lane is not None:
            #cv2.line(image, lane[0], lane[1], (255, 150, 0), 10)
            pass
        
        return image

    
    def driving(self, white_lane, yellow_lane, img):
        # Define control parameters
        kp = 0.02  # Proportional gain
        max_vel = 0.1  # Constant velocity
        velocity = max_vel
        steering_angle = 0.0

        if white_lane is not None and yellow_lane is not None:
            # Calculate the center of the lane
            
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

            velocity, steering_angle, img = self.clac_driving_parameters(max_vel, kp, deviation, img, center_lane)

        elif white_lane is None and yellow_lane is not None:
            y1, y2 = yellow_lane

            deviation = (y2[0] - y1[0])/2
            if deviation < 0:
                deviation = 0
                self.first = True
            
            center_lane = ((int((self.window_width + deviation)/2), int(self.window_height/2)),(int(self.window_width/2), int(self.window_height)))

            velocity, steering_angle, img = self.clac_driving_parameters(max_vel, kp, deviation, img, center_lane)

        elif yellow_lane is None and white_lane is not None:
            w1, w2 = white_lane

            deviation = (w2[0] - w1[0])/2
            if deviation > 0:
                deviation = 0
                self.first = True
            
            center_lane = ((int((self.window_width + deviation)/2), int(self.window_height/2)),(int(self.window_width/2), int(self.window_height)))

            velocity, steering_angle, img = self.clac_driving_parameters(max_vel, kp, deviation, img, center_lane)

        else:
            steering_angle = 0.0
            velocity = max_vel/2

            center_lane = ((int(self.window_width/2), int(self.window_height/1.5)),(int(self.window_width/2), int(self.window_height)))

            # Draw the center lane on the image
            cv2.line(img, center_lane[0], center_lane[1], (0, 0, 255), 5)



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

    
    def clac_driving_parameters(self, max_vel, kp, deviation, img, center_lane):
        velocity = max_vel
        steering_angle = 0.0
        
        # Draw the center lane on the image
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

        return velocity, steering_angle, img
    

    def left_turn(self):
        z, w = self.get_location()
        angle = self.get_angle(z, w)

        while (angle -90) < self.get_angle(z, w):
            # Create Twist message with desired linear and angular velocities
            twist_msg = Twist()
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = 1.0

            # Publish the twist message to control the TurtleBot3
            self.pub_vel_cmd.publish(twist_msg)

        pass


    def right_turn(self):
        z, w = self.get_location()
        angle = self.get_angle(z, w)

        while (angle +90) > self.get_angle(z, w):
            # Create Twist message with desired linear and angular velocities
            twist_msg = Twist()
            twist_msg.linear.x = 0.1
            twist_msg.angular.z = -1.0

            # Publish the twist message to control the TurtleBot3
            self.pub_vel_cmd.publish(twist_msg)

        pass

    
    def hard_left_turn(self):
        z, w = self.get_location()
        angle = self.get_angle(z, w)

        while (angle -90) < self.get_angle(z, w):
            # Create Twist message with desired linear and angular velocities
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 1.0

            # Publish the twist message to control the TurtleBot3
            self.pub_vel_cmd.publish(twist_msg)

        pass


    def hard_right_turn(self):
        z, w = self.get_location()
        angle = self.get_angle(z, w)

        while (angle +90) > self.get_angle(z, w):
            # Create Twist message with desired linear and angular velocities
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = -1.0

            # Publish the twist message to control the TurtleBot3
            self.pub_vel_cmd.publish(twist_msg)

        pass
    

    def get_lot(self):
        # get free parking lot
        get_free_lot = GetFreeSlot()
        rclpy.spin_once(get_free_lot)
        free_lot = get_free_lot.get_free()
        get_free_lot.destroy_node()

        return free_lot
    

    def get_angle(self, z, w):
        # 57.3 deg are 1 rad
        if z < 0:
            angle = 360 - acos(w) * 2 * 57.3
        else:
            angle = acos(w) * 2 * 57.3

        return angle
    

    def get_location(self):
        # get current loaction
        get_current_pose = PositionListener()
        rclpy.spin_once(get_current_pose)
        position = get_current_pose.get_position()
        get_current_pose.destroy_node()

        return position.pose.orientation.z, position.pose.orientation.w


    
def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
