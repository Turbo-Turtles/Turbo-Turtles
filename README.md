# Robotics Project 2023

Project by Georg Forberger, Marvin Menzel, Robert Schneider \
Teamname: Turbo-Turtles

## Overview

In this project, our goal is to program a TurtleBot3 'burger' to drive without a human operator. \
To achieve this, we use ROS2 Humble in combination with python and c++.

## Team

| Team Member  | General Field of Activity |
| ------------- | ------------- |
| Georg Forberger  | Lane Following  |
| Marvin Menzel  | Sign Detection  |
| Robert Schneider  | LiDAR Navigation  |

## Timetable

The schedule is shown [here](https://github.com/orgs/Turbo-Turtles/projects/4/views/2). \
Our weekly progress can be found [here](WEEKLY.md).

## Objectives

#### <u>ROS-Nodes:</u>

- ### __Movement__
  - takes inputs from multiple nodes and combines them to one movement comand \
  -> sends that comand to the TurtleBot
- ### __Camera Vision__
  - recognize :
    - traffic signs
    - lane markings (left: yellow, right: white)
    - parking lot
    - *maybe:* obstacles, other bots
- ### __LiDAR Vision__
  - detect obstacles
  - guide through:
    - dark areas (like the tunnel)
    - cunstruction site (obstacles on lane)

To achieve a fully autonomous driving bot, we need to implement an automatic transition between the camera-based and LiDAR-driven modes.

---
#### <u>Missions:</u>
[Here](SENSOR_USAGE.md) is how the sensors are used in each mission.

### 0. Base mission
- follow lane markings (straight, curves, intersections?, zig zag)
- look out for section signs (can be only activated after prior mission success)

### 1. Traffic mission
traffic light
- search traffic light
- detect traffic light
- red/orange: stop
- green: go

### 2. Intersection mission
left or right
- search traffic sign
- detect traffic sign
- left arrow: turn left procedure
- right arrow: turn right procedure

### 3. Construction mission
obstacle avoidance
- map obstacles with SLAM
- avoidance procedure:
  - use LiDAR navigation2 stack to follow a path through the obstacles

### 4. Parking mission
park on free slot
- search traffic sign
- detect traffic sign
- left arrow: turn left procedure
- drive to parking slots/markings
- search other bot/obstacle
- detect other bot/obstacle
- park on other slot (to be specified)
- reenter main lane (to be specified)

### 5. Level Cross mission
barrier / blocked lane
- section sign is equal to traffic sign/gate
- if gate is closed: stop in front of gate
- if gate is (fully) open: continue driving

### 6. Tunnel mission
enter and leave tunnel
- map tunnel with SLAM
- use LiDAR navigation2 stack to drive to exit
