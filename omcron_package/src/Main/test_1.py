#!/usr/bin/env python3

# -- Import necessary Python packages
import rospy, rosbag, os, sys, threading
from sensor_msgs.msg import Image


# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

# from Classes.GripperBaseClass import Gripper
from OmcronBaseClass.UR3e import*
from OmcronBaseClass.CameraBaseClass import*

# Setup the ROS Node:
rospy.init_node('robot_node')

# Setup the Robot:
robot = UR3e()

# Setup environment:
# env = swift.Swift()
# env.launch(realtime=True)
# env._send_socket
# env.add(robot.model)
# env.add(robot._cam)
# env.add(robot._gripper)

# Setup the initial position of the robot:
# robot.model.q = [-pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0]


# Setup the Camera:
camera = Camera()

point_array = []

detected_objects_blue = []

while not detected_objects_blue:
    detected_objects_blue = camera.detect_object('blue')

for K in detected_objects_blue:
    # Convert to 3D point
    x, y, depth, label = K
    
    point = camera.project_2D_to_3D(x, y, depth)
    point_array.append(point)

    # print(T)
    # print(point)

# # ////////////RED////////////
# detected_objects_red = []

# while not detected_objects_red:
#     detected_objects_red = camera.detect_object('red')

# for K in detected_objects_red:
#     # Convert to 3D point
#     x, y, depth, label = K
    
#     point = camera.project_2D_to_3D(x, y, depth)

#     # print(T)
#     print(point)


# # ////////////yellow///////////
# detected_objects_yellow = []

# while not detected_objects_yellow:
#     detected_objects_yellow = camera.detect_object('yellow')

# for K in detected_objects_yellow:
#     # Convert to 3D point
#     x, y, depth, label = K
    
#     point = camera.project_2D_to_3D(x, y, depth)
#     point_array.append(point)

#     # print(T)
#     # print(point)

for i in range(len(point_array)):
    point_array[i][2] = point_array[i][2] - 0.18
print(point_array)

# robot.move_ee_up_down(env, delta_x=, delta_y=dy, delta_z=dz,real_robot=False)

rospy.spin()
