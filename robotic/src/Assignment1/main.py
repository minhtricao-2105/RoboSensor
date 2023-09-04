#!/usr/bin/env python3
import sys
import os
from onrobot_rg_control.msg import OnRobotRGOutput

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

from Classes.GripperBaseClass import Gripper
from Classes.UR3e import*
from hardcode import*

# Setup the ROS Node:
rospy.init_node('robot_node')
pub   = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

# Setup the Robot:
robot = UR3e()

# Setup the Gripper:
gripper = Gripper()

# Setup the Swift Environment:
env = swift.Swift()
env.launch(realtime=True)
env._send_socket
env.add(robot.model)

# Set the initial position of the robot:
home_position = [0, -pi/2, -pi/2, -pi/2, pi/2, 0]
robot.model.q = copy.deepcopy(home_position)

# Set the initial position of the brick:
brick_locations = brick_locations()

# Set the placing position of the brick:
brick_droppings = brick_dropping()

# # Using MoveIt to plan the robot to home position:
robot.set_up_moveIt(0.1)
robot.arm.go(home_position)

# Open the Gripper:
pub.publish(gripper.OpenGripper())

# Perform the pick and place:
for i in range(len(brick_locations)):

    # Generate the trajectory to pick up the first brick
    robot.move_jtraj(robot.model.q, brick_locations[i], env=env, speed = 4, real_robot = True)

    # Move Robot Down to pick up the brick:
    robot.move_ee_up_down(-0.075, env, 1, real_robot = True)

    # Close the Gripper:
    pub.publish(gripper.MoveGripperToPosition(position = 530))

    # Move Robot up:
    robot.move_ee_up_down(0.14, env, 1, real_robot = True)

    # Move the desired placing position:
    robot.move_jtraj(robot.model.q, brick_droppings[i], env = env, speed = 3, real_robot = True)

    remainder = i % 3

    if remainder == 0:
        robot.move_ee_up_down(-0.12, env, 1, real_robot=True)
        height = 0.12
    elif remainder == 1:
        robot.move_ee_up_down(-0.045, env, 1, real_robot=True)
        height = 0.045
    else:
        robot.move_ee_up_down(0, env, 1, real_robot=True)
        height = 0

    # Open the Gripper:
    pub.publish(gripper.OpenGripper())

    # Move Robot up:
    robot.move_ee_up_down(height, env, 1, real_robot = True)


env.hold