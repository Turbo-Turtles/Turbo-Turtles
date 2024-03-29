#!/usr/bin python3
# -*- coding: utf-8 -*-

import rclpy
import cv2
import array
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, Float64
from sensor_msgs.msg import Image, CompressedImage
#--+--from dynamic_reconfigure.server import Server
#--+--from turtlebot3_autorace_detect.cfg import DetectLaneParamsConfig

class DetectLane(Node):

    def nothing(self):
        pass


    # Create a window
    cv2.namedWindow('crop_image')

    # create trackbars for image cropping
    cv2.createTrackbar('p1_x','crop_image',0,100,nothing)
    cv2.createTrackbar('p2_x','crop_image',0,100,nothing)
    cv2.createTrackbar('p12_y','crop_image',0,100,nothing)
    cv2.createTrackbar('p3_x','crop_image',0,100,nothing)
    cv2.createTrackbar('p4_x','crop_image',0,100,nothing)

    # Set default value for image cropping trackbars.
    cv2.setTrackbarPos('p12_y', 'crop_image', 60)
    cv2.setTrackbarPos('p2_x', 'crop_image', 100)
    cv2.setTrackbarPos('p4_x', 'crop_image', 100)



    def __init__(self):
        super().__init__("lane_detection_node_1")

        """
        #Parameter declaration
        self.hue_white_l = 60
        self.hue_white_h = 120
        self.saturation_white_l = 0
        self.saturation_white_h = 70
        self.lightness_white_l = 230
        self.lightness_white_h = 255

        self.hue_yellow_l = 20
        self.hue_yellow_h = 65
        self.saturation_yellow_l = 40
        self.saturation_yellow_h = 120
        self.lightness_yellow_l = 90
        self.lightness_yellow_h = 190
        """

        #Parameter declaration
        self.hue_white_l = 0
        self.hue_white_h = 4
        self.saturation_white_l = 0
        self.saturation_white_h = 4
        self.lightness_white_l = 150
        self.lightness_white_h = 255

        self.hue_yellow_l = 28
        self.hue_yellow_h = 32
        self.saturation_yellow_l = 230
        self.saturation_yellow_h = 255
        self.lightness_yellow_l = 150
        self.lightness_yellow_h = 255

        self.is_calibration_mode = True

        # subscribes compressed image
        self.sub_image_original = self.create_subscription(Image, '/camera/image_raw', self.cbFindLane, 1)
        
        # publishes lane image in compressed type 
        self.pub_image_lane = self.create_publisher(CompressedImage, '/detect/image_output/compressed', 1)

        if self.is_calibration_mode == True:
            # publishes lane image in compressed type 
            self.pub_image_white_lane = self.create_publisher(CompressedImage, '/detect/image_output_sub1/compressed', 1)
            self.pub_image_yellow_lane = self.create_publisher(CompressedImage, '/detect/image_output_sub2/compressed', 1)
            
        self.pub_lane = self.create_publisher(Float64, '/detect/lane', 1)

        self.publisher = self.create_publisher(Twist, "cmd_vel", 1) #create movement value publisher

        # subscribes state : yellow line reliability
        self.pub_yellow_line_reliability = self.create_publisher(UInt8, '/detect/yellow_line_reliability', 1)
 
        # subscribes state : white line reliability
        self.pub_white_line_reliability = self.create_publisher(UInt8, '/detect/white_line_reliability', 1)
 
        self.cvBridge = CvBridge()

        self.counter = 1

        self.reliability_white_line = 100
        self.reliability_yellow_line = 100


    def cbFindLane(self, image_msg):
        
        # Convert the image to a read-only bytes-like object
        cv_image_ = self.cvBridge.imgmsg_to_cv2(image_msg, "bgr8")

        height, width = cv_image_.shape[:2]
        self.window_width = width
        self.window_height = height
        self.window_size = width * height

        cv_image = self.crop_image(cv_image_)
        
        # find White and Yellow Lanes
        white_fraction, cv_white_lane = self.maskWhiteLane(cv_image)
        yellow_fraction, cv_yellow_lane = self.maskYellowLane(cv_image)

        try:
            if yellow_fraction > (int(self.window_size/200)):
                self.left_fitx, self.left_fit = self.fit_from_lines(self.left_fit, cv_yellow_lane)
                self.mov_avg_left = np.append(self.mov_avg_left,np.array([self.left_fit]), axis=0)

            if white_fraction > (int(self.window_size/200)):
                self.right_fitx, self.right_fit = self.fit_from_lines(self.right_fit, cv_white_lane)
                self.mov_avg_right = np.append(self.mov_avg_right,np.array([self.right_fit]), axis=0)
        except:
            if yellow_fraction > (int(self.window_size/200)):
                self.left_fitx, self.left_fit = self.sliding_windown(cv_yellow_lane, 'left')
                self.mov_avg_left = np.array([self.left_fit])

            if white_fraction > (int(self.window_size/200)):
                self.right_fitx, self.right_fit = self.sliding_windown(cv_white_lane, 'right')
                self.mov_avg_right = np.array([self.right_fit])

        MOV_AVG_LENGTH = 5

        self.left_fit = np.array([np.mean(self.mov_avg_left[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                  np.mean(self.mov_avg_left[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                  np.mean(self.mov_avg_left[::-1][:, 2][0:MOV_AVG_LENGTH])])
        self.right_fit = np.array([np.mean(self.mov_avg_right[::-1][:, 0][0:MOV_AVG_LENGTH]),
                                   np.mean(self.mov_avg_right[::-1][:, 1][0:MOV_AVG_LENGTH]),
                                   np.mean(self.mov_avg_right[::-1][:, 2][0:MOV_AVG_LENGTH])])

        if self.mov_avg_left.shape[0] > 1000:
            self.mov_avg_left = self.mov_avg_left[0:MOV_AVG_LENGTH]

        if self.mov_avg_right.shape[0] > 1000:
            self.mov_avg_right = self.mov_avg_right[0:MOV_AVG_LENGTH]

        driving_direction = self.make_lane(cv_image, white_fraction, yellow_fraction)
        self.driving(driving_direction)


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


    def maskWhiteLane(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)    

        Hue_l = self.hue_white_l
        Hue_h = self.hue_white_h
        Saturation_l = self.saturation_white_l
        Saturation_h = self.saturation_white_h
        Lightness_l = self.lightness_white_l
        Lightness_h = self.lightness_white_h

        # define range of white color in HSV
        lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only white colors
        mask = cv2.inRange(hsv, lower_white, upper_white)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)

        if self.is_calibration_mode == False:
            if fraction_num > (int(self.window_size/17)):
                if self.lightness_white_l < 250:
                    self.lightness_white_l += 5
            elif fraction_num < (int(self.window_size/120)):
                if self.lightness_white_l > 50:
                    self.lightness_white_l -= 5

        how_much_short = 0

        for i in range(0, self.window_height):
            if np.count_nonzero(mask[i,::]) > 0:
                how_much_short += 1

        how_much_short = self.window_height - how_much_short

        if how_much_short > (int(self.window_height/6)):
            if self.reliability_white_line >= 5:
                self.reliability_white_line -= 5
        elif how_much_short <= (int(self.window_height/6)):
            if self.reliability_white_line <= 99:
                self.reliability_white_line += 5

        msg_white_line_reliability = UInt8()
        msg_white_line_reliability.data = self.reliability_white_line
        self.pub_white_line_reliability.publish(msg_white_line_reliability)

        if self.is_calibration_mode == True:
            # publishes white lane filtered image in compressed type
            self.pub_image_white_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))
            cv2.imshow("mask_Result_w", mask)
            cv2.waitKey(1)

        return fraction_num, mask
    

    def maskYellowLane(self, image):
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        Hue_l = self.hue_yellow_l
        Hue_h = self.hue_yellow_h
        Saturation_l = self.saturation_yellow_l
        Saturation_h = self.saturation_yellow_h
        Lightness_l = self.lightness_yellow_l
        Lightness_h = self.lightness_yellow_h

        # define range of yellow color in HSV
        lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
        upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

        # Threshold the HSV image to get only yellow colors
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(image, image, mask = mask)

        fraction_num = np.count_nonzero(mask)
        
        if self.is_calibration_mode == False:
            if fraction_num > (int(self.window_size/17)):
                if self.lightness_yellow_l < 250:
                    self.lightness_yellow_l += 20
            elif fraction_num < (int(self.window_size/120)):
                if self.lightness_yellow_l > 90:
                    self.lightness_yellow_l -= 20

        how_much_short = 0

        for i in range(0, self.window_height):
            if np.count_nonzero(mask[i,::]) > 0:
                how_much_short += 1
        
        how_much_short = self.window_height - how_much_short

        if how_much_short > (int(self.window_height/6)):
            if self.reliability_yellow_line >= 5:
                self.reliability_yellow_line -= 5
        elif how_much_short <= (int(self.window_height/6)):
            if self.reliability_yellow_line <= 99:
                self.reliability_yellow_line += 5

        msg_yellow_line_reliability = UInt8()
        msg_yellow_line_reliability.data = self.reliability_yellow_line
        self.pub_yellow_line_reliability.publish(msg_yellow_line_reliability)

        if self.is_calibration_mode == True:
            # publishes yellow lane filtered image in compressed type
            self.pub_image_yellow_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(mask, "jpg"))
            cv2.imshow("mask_Result_y", mask)
            cv2.waitKey(1)

        return fraction_num, mask
    

    def fit_from_lines(self, lane_fit, image):
        nonzero = image.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])
        margin = 100
        lane_inds = ((nonzerox > (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] - margin)) & (
        nonzerox < (lane_fit[0] * (nonzeroy ** 2) + lane_fit[1] * nonzeroy + lane_fit[2] + margin)))

        # Again, extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        lane_fit = np.polyfit(y, x, 2)

        # Generate x and y values for plotting
        ploty = np.linspace(0, image.shape[0] - 1, image.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]
            
        return lane_fitx, lane_fit
    

    def sliding_windown(self, img_w, left_or_right):
        histogram = np.sum(img_w[int(img_w.shape[0] / 2):, :], axis=0)

        # Create an output image to draw on and visualize the result
        out_img = np.dstack((img_w, img_w, img_w)) * 255

        # Find the peak of the left and right halves of the histogram
        # These will be the starting point for the left and right lines
        midpoint = np.int(histogram.shape[0] / 2)

        if left_or_right == 'left':
            lane_base = np.argmax(histogram[:midpoint])
        elif left_or_right == 'right':
            lane_base = np.argmax(histogram[midpoint:]) + midpoint

        # Choose the number of sliding windows
        nwindows = 20

        # Set height of windows
        window_height = np.int(img_w.shape[0] / nwindows)

        # Identify the x and y positions of all nonzero pixels in the image
        nonzero = img_w.nonzero()
        nonzeroy = np.array(nonzero[0])
        nonzerox = np.array(nonzero[1])

        # Current positions to be updated for each window
        x_current = lane_base

        # Set the width of the windows +/- margin
        margin = 50

        # Set minimum number of pixels found to recenter window
        minpix = 50

        # Create empty lists to receive lane pixel indices
        lane_inds = []

        # Step through the windows one by one
        for window in range(nwindows):
            # Identify window boundaries in x and y
            win_y_low = img_w.shape[0] - (window + 1) * window_height
            win_y_high = img_w.shape[0] - window * window_height
            win_x_low = x_current - margin
            win_x_high = x_current + margin

            # Draw the windows on the visualization image
            cv2.rectangle(out_img, (win_x_low, win_y_low), (win_x_high, win_y_high), (0, 255, 0), 2)

            # Identify the nonzero pixels in x and y within the window
            good_lane_inds = ((nonzeroy >= win_y_low) & (nonzeroy < win_y_high) & (nonzerox >= win_x_low) & (
                nonzerox < win_x_high)).nonzero()[0]

            # Append these indices to the lists
            lane_inds.append(good_lane_inds)

            # If you found > minpix pixels, recenter next window on their mean position
            if len(good_lane_inds) > minpix:
                x_current = np.int(np.mean(nonzerox[good_lane_inds]))

        # Concatenate the arrays of indices
        lane_inds = np.concatenate(lane_inds)

        # Extract line pixel positions
        x = nonzerox[lane_inds]
        y = nonzeroy[lane_inds]

        # Fit a second order polynomial to each
        try:
            lane_fit = np.polyfit(y, x, 2)
            self.lane_fit_bef = lane_fit
        except:
            lane_fit = self.lane_fit_bef

        # Generate x and y values for plotting
        ploty = np.linspace(0, img_w.shape[0] - 1, img_w.shape[0])
        lane_fitx = lane_fit[0] * ploty ** 2 + lane_fit[1] * ploty + lane_fit[2]

        return lane_fitx, lane_fit
    

    def make_lane(self, cv_image, white_fraction, yellow_fraction):
        # Create an image to draw the lines on
        warp_zero = np.zeros((cv_image.shape[0], cv_image.shape[1], 1), dtype=np.uint8)

        color_warp = np.dstack((warp_zero, warp_zero, warp_zero))
        color_warp_lines = np.dstack((warp_zero, warp_zero, warp_zero))

        ploty = np.linspace(0, cv_image.shape[0] - 1, cv_image.shape[0])

        if yellow_fraction > (int(self.window_size/200)):
            pts_left = np.array([np.flipud(np.transpose(np.vstack([self.left_fitx, ploty])))])
            cv2.polylines(color_warp_lines, np.int_([pts_left]), isClosed=False, color=(0, 0, 255), thickness=25)

        if white_fraction > (int(self.window_size/200)):
            pts_right = np.array([np.transpose(np.vstack([self.right_fitx, ploty]))])
            cv2.polylines(color_warp_lines, np.int_([pts_right]), isClosed=False, color=(255, 255, 0), thickness=25)
        
        self.is_center_x_exist = True

        if self.reliability_white_line > 50 and self.reliability_yellow_line > 50:   
            if white_fraction > (int(self.window_size/200)) and yellow_fraction > (int(self.window_size/200)):
                centerx = np.mean([self.left_fitx, self.right_fitx], axis=0)
                pts = np.hstack((pts_left, pts_right))
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])
                #self.get_logger().info(str(pts_center))

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

                # Draw the lane onto the warped blank image
                cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))

            if white_fraction > (int(self.window_size/200)) and yellow_fraction <= (int(self.window_size/200)):
                centerx = np.subtract(self.right_fitx, 320)
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

            if white_fraction <= (int(self.window_size/200)) and yellow_fraction > (int(self.window_size/200)):
                centerx = np.add(self.left_fitx, 320)
                pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

                cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        elif self.reliability_white_line <= 50 and self.reliability_yellow_line > 50:
            centerx = np.add(self.left_fitx, 320)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        elif self.reliability_white_line > 50 and self.reliability_yellow_line <= 50:
            centerx = np.subtract(self.right_fitx, 320)
            pts_center = np.array([np.transpose(np.vstack([centerx, ploty]))])

            cv2.polylines(color_warp_lines, np.int_([pts_center]), isClosed=False, color=(0, 255, 255), thickness=12)

        else:
            self.is_center_x_exist = False
            # TODO: stop
            pass

        # Combine the result with the original image
        final = cv2.addWeighted(cv_image, 1, color_warp, 0.2, 0)
        final = cv2.addWeighted(final, 1, color_warp_lines, 1, 0)

        if self.is_center_x_exist == True:
            # publishes lane center
            msg_desired_center = Float64()
            msg_desired_center.data = centerx.item(int(self.window_height * 0.6))
            self.pub_lane.publish(msg_desired_center)
            #self.get_logger().info(str(int(msg_desired_center.data)) + '; ' + str(int(self.window_height * 0.6)))

        self.pub_image_lane.publish(self.cvBridge.cv2_to_compressed_imgmsg(final, "jpg"))
        cv2.imshow("Lane_Detection_Result", final)
        cv2.waitKey(1)

        #return (int(centerx.item(int(self.window_height * 0.6))), int(self.window_height * 0.6))
        return (int(centerx.item(int(self.window_height * 0.6))), int(self.window_height * 0.6)),(int(centerx.item(int(self.window_height)-1)), int(self.window_height))

    
    def driving(self, driving_direction):
        pt1, pt2 = driving_direction
        
        if pt1 is not None and pt2 is not None:
            # Calculate the steering angle
            if (pt2[0]-pt1[0]) is not 0:
                deviation = (pt2[1]-pt1[1]/pt2[0]-pt1[0])
            else:
                deviation = 0

            # Define control parameters
            kp = 0.1  # Proportional gain
            velocity = 0.08  # Constant velocity

            # Calculate the desired steering angle based on the deviation and control parameters
            steering_angle = kp * deviation / 200      #<------- winkel zu klein und zu groß raußfiltern
            self.get_logger().info(str(steering_angle))

            # Create Twist message with desired linear and angular velocities
            twist_msg = Twist()
            twist_msg.linear.x = velocity
            twist_msg.angular.z = steering_angle

            # Publish the twist message to control the TurtleBot3
            self.publisher.publish(twist_msg)




def main(args=None):
    rclpy.init(args=args)
    node = DetectLane()
    rclpy.spin(node)

    rclpy.shutdown()

if __name__=='__main__':
    main()