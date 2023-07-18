from lidar_navigation.position_listener import PositionListener

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

from geometry_msgs.msg import PoseStamped

from math import sqrt, cos, acos, sin

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
    def two_pass(self, data, M, N, x1, y1, x2, y2):
        linked = []
        labels = np.zeros((M,N))
        nextLabel = 1

        # first pass
        for row in range(y1, y2+1):
            for column in range(x1, x2+1):
                # if not background
                if data[row][column].tolist() != self.background:
                    # get all neighbors (W, N, NW, NE)
                    neighbors = []
                    for direction in self.adjacent:
                        if row+direction[0] in range(y1, y2+1) and column+direction[1] in range(x1, x2+1):
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
    def create_waypoints(self, obstacles, x, y, x_a, y_a, x_b, y_b):
        waypoints = []

        # calculate the distances between the line of sight and the obstacles
        dists = []
        for coord in obstacles:
            dists.append(self.dist(x, y, x_a, y_a, coord[1], coord[0]))

        # sort obstacle coordinates from closest to farthest away from robot sight line (0°)
        obstacles = np.asarray(obstacles)[np.argsort(np.asarray(dists))]

        # f1 - dist on a to first and third obstacle
        f1 = ( self.dist(x, y, x_b, y_b, obstacles[0][1], obstacles[0][0]) + self.dist(x, y, x_b, y_b, obstacles[2][1], obstacles[2][0]) ) / 2

        # f2 - dist on a to second obstacle
        f2 = self.dist(x, y, x_b, y_b, obstacles[1][1], obstacles[1][0])

        # h1 - dist on b to first obstacle
        h1 = sorted(dists)[0]

        # h2 - dist on b to second obstacle
        h2 = sorted(dists)[1]

        # h3 - dist on b to third obstacle
        h3 = sorted(dists)[2]

        # normalize a-vector
        a_lenght = sqrt( (x_a-x)**2 + (y_a-y)**2 )
        norm_a_x = (x_a-x) / a_lenght
        norm_a_y = (y_a-y) / a_lenght

        # normalize b-vector
        b_lenght = sqrt( (x_b-x)**2 + (y_b-y)**2 )
        norm_b_x = (x_b-x) / b_lenght
        norm_b_y = (y_b-y) / b_lenght


        # waypoint 1
        x1 = x + norm_a_x * (f1 + 1) + norm_b_x * 0
        y1 = y + norm_a_y * f1 + norm_b_y * 0
        waypoints.append([x1, y1])

        # waypoint 2
        x2 = x + norm_a_x * (f2 + 1) + norm_b_x * h1
        y2 = y + norm_a_y * f2 + norm_b_y * h1
        waypoints.append([x2, y2])

        # waypoint 3
        x3 = x + norm_a_x * (f1 - 1) + norm_b_x * h2
        y3 = y + norm_a_y * f1 + norm_b_y * h2
        waypoints.append([x3, y3])

        # waypoint 4
        x4 = x + norm_a_x * (f2 + 1) + norm_b_x * h3
        y4 = y + norm_a_y * f2 + norm_b_y * h3
        waypoints.append([x4, y4])

        # waypoint 5
        x5 = x + norm_a_x * (f1 + 1) + norm_b_x * (h3 + h1)
        y5 = y + norm_a_y * f1 + norm_b_y * (h3 + h1)
        waypoints.append([x5, y5])

        return waypoints
    
    def convert_waypoints(self, waypoints):
        for waypoint in range(len(waypoints)):
            waypoints[waypoint][0] = self.map.info.origin.position.y + waypoints[waypoint][0] * self.map.info.resolution
            waypoints[waypoint][1] = self.map.info.origin.position.x + waypoints[waypoint][1] * self.map.info.resolution

        return waypoints
    
    def get_rect(x, y, angle, a, b):
        x_a = x + a * cos(angle / 57.3)
        y_a = y - a * sin(angle / 57.3)

        x_b = x + b * cos((angle + 90) / 57.3)
        y_b = y - b * sin((angle + 90) / 57.3)

        x_ab = x + a * cos(angle / 57.3) + b * cos((angle + 90) / 57.3)
        y_ab = y - a * sin(angle / 57.3) - b * sin((angle + 90) / 57.3)

        return x, y, x_a, y_a, x_b, y_b, x_ab, y_ab

    def get_big_rect(x, y, x_a, y_a, x_b, y_b, x_ab, y_ab):
        x_min = int(min(x, x_a, x_b, x_ab))
        x_max = int(max(x, x_a, x_b, x_ab))

        y_min = int(min(y, y_a, y_b, y_ab))
        y_max = int(max(y, y_a, y_b, y_ab))

        return x_min, y_min, x_max, y_max

    def check_in_rect(point, rect):
        # (0 < AM*AB < AB*AB) and (0 < AM*AD < AD*AD)
        # ->
        # (0 < (p_x-x)*(x_a-x) + (p_y-y)*(y_a-y) < (x_a-x)*(x_a-x) + (y_a-y)*(y_a-y)) and (0 < (p_x-x)*(x_b-x) + (p_y-y)*(y_b-y) < (x_b-x)*(x_b-x) + (y_b-y)*(y_b-y))

        # point coords
        p_x, p_y = point

        # rect coords
        x, y, x_a, y_a, x_b, y_b, x_ab, y_ab = rect

        if (0 < (p_x-x)*(x_a-x) + (p_y-y)*(y_a-y) < (x_a-x)*(x_a-x) + (y_a-y)*(y_a-y)) and (0 < (p_x-x)*(x_b-x) + (p_y-y)*(y_b-y) < (x_b-x)*(x_b-x) + (y_b-y)*(y_b-y)):
            return 1

        else:
            return 0

    def dist(x, y, x_a, y_a, p_x, p_y):
        angle = acos( ( (p_x-x)*(x_a-x) + (p_y-y)*(y_a-y) ) / ( sqrt( (p_x-x)**2 + (p_y-y)**2 ) * sqrt( (x_a-x)**2 + (y_a-y)**2 ) ) )

        d = sqrt( (p_x-x)**2 + (p_y-y)**2 ) * sin(angle)

        return d
###################################################################


    def get_location(self):
        # get current loaction
        get_current_pose = PositionListener()
        rclpy.spin_once(get_current_pose)
        position = get_current_pose.get_position()
        get_current_pose.destroy_node()

        return position.pose.position.x, position.pose.position.y, position.pose.orientation.z, position.pose.orientation.w
