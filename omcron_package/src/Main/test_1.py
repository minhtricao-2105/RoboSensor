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
env = swift.Swift()
env.launch(realtime=True)
env._send_socket
env.add(robot.model)

# Setup the initial position of the robot:
robot.model.q = [-pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0]

# Setup the Camera:
camera = Camera()

K = None

while K is None:
    K = camera.detect_blue_object()

# convert to 3D point

x,y, depth = K

point_3D = camera.project_2D_to_3D(x, y, depth)

print(point_3D)

# Get the end-effector pose:
T = robot.model.fkine(robot.model.q).A


dx = T[0, 3] - point_3D[0]/1000
dy = T[1, 3] - point_3D[1]/1000
dz = T[2, 3] - point_3D[2]/1000

print('Detected Object at: ', dx, dy, dz)

# robot.move_ee_up_down(env, delta_x=dx, delta_y=dy, delta_z=dz,real_robot=False)

rospy.spin()
