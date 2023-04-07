# Robotics Project 2023

Project by Georg Forberger, Marvin Menzel, Robert Schneider \
Teamname: Turbo-Turtles

# Overview

In this project, our goal is to program a TurtleBot3 'burger' to drive without a human operator. \
To achieve this, we use ROS2 Humble in combination with python and c++.

## Timetable

The schedule will be shown [here](https://github.com/Marvinissimus/Turbo-Turtles/projects) soon.

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