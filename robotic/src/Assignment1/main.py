#!/usr/bin/env python3
import sys
import os

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

from Classes.GripperBaseClass import Gripper
from Classes.UR3e import*
from hardcode import*

# Setup the ROS Node:
rospy.init_node('robot_node')

# Setup the Robot:
robot = UR3e()

# Setup the Gripper:
# gripper = Gripper()

# Setup the Swift Environment:
env = swift.Swift()
env.launch(realtime=True)
env._send_socket
env.add(robot.model)

robot.model.q = [0, -pi/2, -pi/2, -pi/2, pi/2, 0]

brick_locations = brick_locations()

brick_droppings = brick_dropping()

# Perform the task:
for i in range(len(brick_locations)):

    # Generate the trajectory to pick up the first brick
    robot.move_jtraj(robot.model.q, brick_locations[i], env=env)

    # Move Robot Down to pick up the brick:
    robot.move_ee_up_down(-0.03, env, 1)

    # Close the Gripper:
    # gripper.MoveGripperToPosition(position = 300)

    # Move Robot up:
    robot.move_ee_up_down(0.03, env, 1)

    # Move the desired placing position:
    robot.move_jtraj(robot.model.q, brick_droppings[i], env = env)

    # Move Robot Down to pick up the brick:
    robot.move_ee_up_down(-0.03, env, 1)

    # Open the Gripper:
    # gripper.MoveGripperToPosition(position = 300)

    # Move Robot up:
    robot.move_ee_up_down(0.03, env, 1)


env.hold