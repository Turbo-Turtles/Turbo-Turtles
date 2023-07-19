import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid

import numpy as np
import matplotlib.pyplot as plt

from math import sqrt, cos, acos, sin

class MapRecognition(Node):
    def __init__(self, angle):
        super().__init__("map_recognition")

        self.angle = angle
        self.a = 22
        self.b = 30

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

    # the returned coordinates can than be specifically used in the receiving scipt
    def get_waypoints(self, x, y):
        # get location
        # x, y = self.get_location()

        # calculate coords of the robot for matplotlib coordinate system
        robot_x, robot_y = (x - self.map.info.origin.position.x) / self.map.info.resolution, (y - self.map.info.origin.position.y) / self.map.info.resolution

        # invert map    
        data = self.prepare_data(self.map.data, self.map.info.height, self.map.info.width)

        # get rect of view field
        x, y, x_a, y_a, x_b, y_b, x_ab, y_ab = self.get_rect(robot_x, robot_y, self.angle, self.a, self.b)

        # optimize view field to be aligned
        x1, y1, x2, y2 = self.get_big_rect(x, y, x_a, y_a, x_b, y_b, x_ab, y_ab)

        # apply two-pass algorithm (label connected areas)
        label_img, label_amount = self.two_pass(data, self.map.info.height, self.map.info.width, x1, y1, x2, y2)

        # get center coords of obstacles
        coords = self.find_coords(label_img, label_amount, x1, y1, x2, y2)   # check in view field

        # check if coords are in view field
        for coord in coords:
            print(coord)
            if self.check_in_rect((coord[1], coord[0]),(x, y, x_a, y_a, x_b, y_b, x_ab, y_ab)):
                data[coord[0]][coord[1]] = self.center
            else:
                coords.remove(coord)

        # set and visualize waypoints
        waypoints = self.create_waypoints(coords, robot_x, robot_y, x_a, y_a, x_b, y_b)

        ###################
        # creates plot of the recognized map
        # after the plot window is closed, the execution starts
        # (can be deleted)

        # paint stuff
        data[int(robot_y)][int(robot_x)] = self.bot

        data[int(robot_y + self.a*sin(self.angle / 57.3))][int(robot_x + self.a*cos(self.angle / 57.3))] = self.bot
        data[int(robot_y + self.b*sin((self.angle + 90) / 57.3))][int(robot_x + self.b*cos((self.angle + 90) / 57.3))] = self.bot

        for coord in coords:
            data[coord[0]][coord[1]] = self.center

        for wp in waypoints:
            data[int(robot_y) + int(wp[1] / self.map.info.resolution)][int(robot_x) + int(wp[0] / self.map.info.resolution)] = self.waypoint
            print(wp)

        plt.imshow(data, interpolation='None')
        ax = plt.gca()
        ax.invert_yaxis()
        plt.show()

        ###################

        return waypoints


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


        # waypoint 1
        a1 = f1
        b1 = 0
        angle1 = 0
        waypoints.append([a1 * self.map.info.resolution + 0.05, b1 * self.map.info.resolution + 0.05, angle1])

        # waypoint 2
        a2 = f2
        b2 = h1
        angle2 = 90
        waypoints.append([a2 * self.map.info.resolution + 0.01, b2 * self.map.info.resolution + 0.05, angle2])

        # waypoint 3
        a3 = f1
        b3 = h2
        angle3 = 90
        waypoints.append([a3 * self.map.info.resolution - 0.08, b3 * self.map.info.resolution + 0.05, angle3])

        # waypoint 4
        a4 = f2
        b4 = h3
        angle4 = 90
        waypoints.append([a4 * self.map.info.resolution + 0.01, b4 * self.map.info.resolution + 0.05, angle4])

        # waypoint 5
        a5 = f1
        b5 = h3 + h1
        angle5 = 180
        waypoints.append([a5 * self.map.info.resolution - 0.2, b5 * self.map.info.resolution + 0.05, angle5])

        return waypoints
    
    def get_rect(self, x, y, angle, a, b):
        x_a = x + a * cos(angle / 57.3)
        y_a = y + a * sin(angle / 57.3)

        x_b = x + b * cos((angle + 90) / 57.3)
        y_b = y + b * sin((angle + 90) / 57.3)

        x_ab = x + a * cos(angle / 57.3) + b * cos((angle + 90) / 57.3)
        y_ab = y + a * sin(angle / 57.3) + b * sin((angle + 90) / 57.3)

        print(x, y, x_a, y_a, x_b, y_b, x_ab, y_ab)

        return x, y, x_a, y_a, x_b, y_b, x_ab, y_ab

    def get_big_rect(self, x, y, x_a, y_a, x_b, y_b, x_ab, y_ab):
        x_min = int(min(x, x_a, x_b, x_ab))
        x_max = int(max(x, x_a, x_b, x_ab))

        y_min = int(min(y, y_a, y_b, y_ab))
        y_max = int(max(y, y_a, y_b, y_ab))

        print(x_min, y_min, x_max, y_max)

        return x_min, y_min, x_max, y_max

    def check_in_rect(self, point, rect):
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

    def dist(self, x, y, x_a, y_a, p_x, p_y):
        angle = acos( ( (p_x-x)*(x_a-x) + (p_y-y)*(y_a-y) ) / ( sqrt( (p_x-x)**2 + (p_y-y)**2 ) * sqrt( (x_a-x)**2 + (y_a-y)**2 ) ) )

        d = sqrt( (p_x-x)**2 + (p_y-y)**2 ) * sin(angle)

        return d