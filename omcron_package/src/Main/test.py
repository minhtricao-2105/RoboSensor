#!/usr/bin/env python3
import sys
import os, copy
from math import pi
import roboticstoolbox as rtb

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

from OmcronBaseClass.BaseClass import OmcronBaseClass

robot = OmcronBaseClass()

robot.model.q = [0/2, 0, 0, 0, 0, 0]

robot.send_position_to_robot(type = 1, position = robot.model.q, velocity = 1.4, acc_time = 0.2, blend_percentage = False, fine_goal = False)