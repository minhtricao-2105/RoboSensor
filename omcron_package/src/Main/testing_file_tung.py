#!/usr/bin/env python3
# -- Import necessary Python packages
import rospy, rosbag, os, sys, threading
from sensor_msgs.msg import Image

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))

# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

# from Classes.GripperBaseClass import Gripper
# from OmcronBaseClass.UR3e import*
from OmcronBaseClass.CameraBaseClass import*
from OmcronBaseClass.Mission import*

# Setup the ROS Node:
rospy.init_node('robot_node')

camera = Camera()
misison = Mission()

misison.detect_multi_object(camera)


rospy.spin()