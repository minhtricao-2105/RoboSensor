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
env.add(robot._cam)
env.add(robot._gripper)

# Setup the initial position of the robot:
robot.model.q = [-pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0]

# Setup the Camera:
camera = Camera()

detected_objects = []

while not detected_objects:
    detected_objects = camera.detect_object()

for K in detected_objects:
    # Convert to 3D point
    x, y, depth = K
    
    point = camera.project_2D_to_3D(x, y, depth)

    # print(T)
    print(point)



# robot.move_ee_up_down(env, delta_x=, delta_y=dy, delta_z=dz,real_robot=False)

rospy.spin()
