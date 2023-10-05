#!/usr/bin/env python3

# -- Import necessary Python packages
import rospy, rosbag, os, sys
from sensor_msgs.msg import Image

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

# from Classes.GripperBaseClass import Gripper
from Classes.UR3e import*
from Classes.UR3eWithRealsense import UR3eWithRealSense

# Setup the ROS Node:
rospy.init_node('robot_node')

# Setup the Robot:
robot = UR3eWithRealSense("captured_data.bag")

# Setup the Swift Environment:
env = swift.Swift()
env.launch(realtime=True)
env._send_socket
env.add(robot.model)

# Set the initial position of the robot:
home_position = [-pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0]
robot.model.q = [-pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0]

# # Using MoveIt to plan the robot to home position:
# robot.set_up_moveIt(0.1)
# robot.arm.go(home_position)


waypoints = robot.generate_path(delta_x=-0.2, steps = 50)

# Begin recording:
for i in range(len(waypoints) - 1): 
    start_point = waypoints[i]
    end_point = waypoints[i + 1]
    path = robot.perform_rmrc_2_points(start_point, end_point, num_waypoints = 2)

    # Move the robot one steps:
    robot.move_simulation_robot(path = path, env = env)

    robot.send_trajectory_to_client(path, speed = 1)

    #delay
    rospy.sleep(0.05)


# After everything is done, close the bag
robot.close_bag()
