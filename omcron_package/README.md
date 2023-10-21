## Description:
In this innovative project, we will use Universal Robot UR3 to perform pick and place task. Initially, we positioned six dice (comprising two blue, two red, and two yellow ones) on the table. To facilitate the process, we integrated an RGB camera to accurately detect the color of each dice, subsequently utilizing image plane coordinates to compute the translation of each dice in the camera's frame. Utilizing the acquired data, we transformed the target points back to the robot's end effector, enabling us to utilize the RMRC (Robot Motion Planning and Control) technique to generate a path from the current pose to the desired pose. In the final stage, leveraging the color information detected by the camera, the UR3 robot efficiently move the dice to pre-defined locations corresponding to their respective colors.

## Installation:
To use this project, first of all, you need to install the Universal Driver. Here is step by step to install the Universal Driver:

## Universal_Robots_ROS_Driver:

### Requirement
This driver requires a system setup with ROS. It is recommended to use Ubuntu 18.04 with ROS melodic, however using Ubuntu 20.04 with ROS noetic should also work.

To make sure that robot control isn't affected by system latencies, it is highly recommended to use a real-time kernel with the system. See the real-time setup guide on information how to set this up.

### How to install:
```
# source global ros
$ source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace (if you already have skip this step and go strathforward to your workplace)
$ mkdir -p catkin_ws/src && cd catkin_ws

# clone the driver
$ git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver

# clone the description. Currently, it is necessary to use the melodic-devel branch.
$ git clone -b melodic-devel https://github.com/ros-industrial/universal_robot.git src/universal_robot

# install dependencies
$ sudo apt update -qq
$ rosdep update
$ rosdep install --from-paths src --ignore-src -y

# build the workspace
$ catkin_make

# activate the workspace (ie: source it)
$ source devel/setup.bash 
```
