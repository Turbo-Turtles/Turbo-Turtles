#!/usr/bin/env python3
# get own path
import os, sys
sys.path.append(os.path.join(
    os.path.dirname(__file__),
    "../../../../../../Turbo-Turtles/lidar_navigation/lidar_navigation/"))

from position_listener import PositionListener

from time import sleep

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PoseStamped


class MapRecognition(Node):
    def __init__(self):
        super().__init__("map_recognition")

        self.threshold = 10
        self.background = [0., 0., 0.]
        self.center = [1., .0, .0]
        self.bot = [.0, 1., .0]
        self.waypoint = [.0, .0, 1.]
        self.adjacent = [[0,-1], [-1,0], [-1,-1], [-1,1]]    # W, N, NW, NE

        self.saved = False

        # quality of service policy
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)

        self.map = OccupancyGrid()

        self.create_subscription(
            OccupancyGrid,
            'map',
            self.mission_callback,
            qos_profile=qos_policy
        )

    def mission_callback(self, msg):
        # save global map
        self.map = msg
        self.saved = True

    def success(self):
        if self.saved:
            # analyze the map
            x, y, resolution = self.map.info.width, self.map.info.height, self.map.info.resolution
            origin = self.map.info.origin
            print(x, y, resolution)
            print(origin.position.x, origin.position.y, origin.orientation.z, origin.orientation.w)
        
            return 1

        else:
            return 0

###################################################################
# maybe this should be in construction_site file and be replaced by get_coords
# the returned coordinates can than be specifically used in the receiving scipt
    def get_waypoints(self):
        x, y, z, w = self.get_location()
        robot_x, robot_y = (x - self.map.info.origin.position.x) / self.map.info.resolution, (y - self.map.info.origin.position.y) / self.map.info.resolution
        data = self.prepare_data(self.map.data, self.map.info.height, self.map.info.width)
        end_img = data
        label_img, label_amount = self.two_pass(data, self.map.info.height, self.map.info.width)

        end_img[int(robot_y)][int(robot_x)] = self.bot

        coords = self.find_coords(label_img, label_amount, robot_x, robot_y, self.map.info.width, self.map.info.height)

        for coord in coords:
            end_img[coord[0]][coord[1]] = self.center

        print(len(coords))
        print(coords)

        waypoints = self.create_waypoints(coords, robot_x, robot_y)

        for wp in waypoints:
            end_img[int(wp[0])][int(wp[1])] = self.waypoint

        plt.imshow(end_img, interpolation='None')
        ax = plt.gca()
        ax.invert_yaxis()
        plt.show()

        return self.convert_waypoints(waypoints)
###################################################################

    def prepare_data(self, data, M, N):
        new_data = np.zeros((M,N,3))
        src = np.zeros((M,N))

        for row in range(M):
            for column in range(N):
                # print(row, column, row*N+column)
                # sleep(.001)
                src[row][column] = data[row*N+column]

        new_data[src<self.threshold] = [0,0,0]
        new_data[src>self.threshold] = [1,1,1]

        return new_data


###################################################################
# two pass algorithm with coords for all obstacles
    def two_pass(self, data, M, N):
        linked = []
        labels = np.zeros((M,N))
        nextLabel = 1

        # first pass
        for row in range(M):
            for column in range(N):
                # if not background
                if data[row][column].tolist() != self.background:
                    # get all neighbors (W, N, NW, NE)
                    neighbors = []
                    for direction in self.adjacent:
                        if row+direction[0] in range(M) and column+direction[1] in range(N):
                            if data[row+direction[0]][column+direction[1]].tolist() != self.background:
                                neighbors.append(labels[row+direction[0]][column+direction[1]])
                    # create new label if no neighbors found
                    if len(neighbors) == 0:
                        linked.append([nextLabel])
                        labels[row][column] = nextLabel
                        nextLabel += 1

                    else:
                        # find smallest label of neighbors
                        labels[row][column] = int(min(neighbors))
                        for label in neighbors:
                            for L in neighbors:
                                if int(L) not in linked[int(label)-1]:
                                    linked[int(label)-1].append(int(L))
                
        # second pass
        for i in range(len(linked)):
            for item in linked[i]:
                for a in linked[item-1]:
                    if a not in linked[i]:
                        linked[i].append(a)

        # clean up
        linked_short = []
        skip = []
        for i in range(len(linked)):
            if i+1 not in skip:
                for a in range(1, len(linked[i])):
                    skip.append(linked[i][a])
                linked_short.append(linked[i])

        # paint
        for i in range(len(linked_short)):
            for item in linked_short[i]:
                labels[labels==item] = i+1

        print(linked)
        print(linked_short)
        
        return labels, len(linked_short) # label_amount
    
    # find the center coordinate of each label
    def find_coords(self, data, labels, x1, y1, x2, y2):
        coords = []
        for i in range(labels):
            coords.append([])

        # get all the same labels coords together
        for row in range(len(data)):
            for column in range(len(data[row])):

                item = data[row][column]
                if item > 0:
                    coords[int(item)-1].append([row, column])

        # get center per label
        for label in range(len(coords)):
            min_x, min_y = self.map.info.height, self.map.info.width
            max_x, max_y = 0, 0
            for coord in coords[label]:
                if coord[0] < min_x:
                    min_x = coord[0]
                if coord[0] > max_x:
                    max_x = coord[0]
                
                if coord[1] < min_y:
                    min_y = coord[1]
                if coord[1] > max_y:
                    max_y = coord[1]
            
            coords[label] = [int((max_x+min_x) / 2), int((max_y+min_y) / 2)+1]

        # check for all center points within x1, y1, x2, y2
        valid_coords = []
        for coord in coords:
            if coord[0]>y1 and coord[0]<y2 and coord[1]>x1 and coord[1]<x2:
                valid_coords.append(coord)


        return valid_coords
###################################################################

###################################################################
# waypoint creation should maybe move to the contruction_site file, since its very specificly for it
    # create waypoints
    def create_waypoints(self, obstacles, robot_x, robot_y):
        waypoints = []
        obstacles = np.asarray(obstacles)

        # prepare and sort the received coords
        np.sort(obstacles, axis=0)  # sort along y axis

        # print(obstacles)
        
        # waypoint 1
        x1 = obstacles[obstacles[:,0].tolist().index(max(obstacles[:,0])), 1] +3
        y1 = robot_y
        waypoints.append([y1, x1])

        # waypoint 2
        x2 = obstacles[obstacles[:,0].tolist().index(sorted(obstacles[:,0])[1]), 1]
        y2 = obstacles[obstacles[:,0].tolist().index(max(obstacles[:,0])), 0]
        waypoints.append([y2, x2])

        # waypoint 3
        x3 = x1 - 1 -3
        y3 = obstacles[obstacles[:,0].tolist().index(sorted(obstacles[:,0])[1]), 0]
        waypoints.append([y3, x3])

        # waypoint 4
        x4 = x2
        y4 = obstacles[obstacles[:,0].tolist().index(min(obstacles[:,0])), 0]
        waypoints.append([y4, x4])

        # waypoint 5
        x5 = x1 -3
        y5 = y4 + (y2 - robot_y)
        waypoints.append([y5, x5])

        # sort waypoints
        print(waypoints)
        waypoints = sorted(waypoints, key=lambda x: x[0])
        print(waypoints)

        return waypoints
    
    def convert_waypoints(self, waypoints):
        for waypoint in range(len(waypoints)):
            waypoints[waypoint][0] = self.map.info.origin.position.y + waypoints[waypoint][0] * self.map.info.resolution
            waypoints[waypoint][1] = self.map.info.origin.position.x + waypoints[waypoint][1] * self.map.info.resolution

        return waypoints
###################################################################


    def get_location(self):
        # get current loaction
        get_current_pose = PositionListener()
        rclpy.spin_once(get_current_pose)
        position = get_current_pose.get_position()
        get_current_pose.destroy_node()

        return position.pose.position.x, position.pose.position.y, position.pose.orientation.z, position.pose.orientation.w
