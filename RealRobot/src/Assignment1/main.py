#!/usr/bin/env python3
import sys
import os
# from onrobot_rg_control.msg import OnRobotRGOutput

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

# from Classes.GripperBaseClass import Gripper
from Classes.UR3e import*
from hardcode import*

# Setup the ROS Node:
rospy.init_node('robot_node')
# pub   = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

# Setup the Robot:
robot = UR3e()

# Setup the Gripper:
# gripper = Gripper()

# Setup the Swift Environment:
env = swift.Swift()
env.launch(realtime=True)
env._send_socket
env.add(robot.model)

# Set the initial position of the robot:
home_position = [0, -pi/2, -pi/2, -pi/2, pi/2, 0]
robot.model.q = [0, -pi/2, -pi/2, -pi/2, pi/2, 0]

# Set the initial position of the brick:
brick_locations = brick_locations()

# Set the placing position of the brick:
brick_droppings = brick_dropping()

# Using MoveIt to plan the robot to home position:
robot.set_up_moveIt(0.1)
robot.arm.go(home_position)

# Open the Gripper:
# pub.publish(gripper.OpenGripper())       

# Perform the pick and place:
for i in range(len(brick_locations)):

    # Generate the trajectory to pick up the first brick
    robot.move_jtraj(robot.model.q, brick_locations[i], env=env, speed = 1, real_robot = True)

    # Move Robot Down to pick up the brick:
    robot.move_ee_up_down(env, delta_z = -0.075, speed = 1, real_robot = True)

    # Close the Gripper:
    # pub.publish(gripper.MoveGripperToPosition(position = 540))

    # Move Robot upL
    robot.move_ee_up_down(env, delta_z = 0.085, speed = 1, real_robot = True)
        
    # Move the desired placing position: 
    robot.move_jtraj(robot.model.q, brick_droppings[i], env = env, speed = 1, real_robot = True)
    
    remainder = i % 3

    if remainder == 0:
        # Move end effector down to place the brick:
        robot.move_ee_up_down(env, delta_z = -0.11, speed = 1, real_robot = True)
        height = 0.12

    elif remainder == 1:
        # Move end effector down to place the brick:
        robot.move_ee_up_down(env, delta_z = -0.045, speed = 1, real_robot = True)
        height = 0.045

    else:
        # Move end effector down to place the brick:
        robot.move_ee_up_down(env, delta_z = -0.02, speed = 1, real_robot = True)
        height = 0.02

    # Open the Gripper:
    # pub.publish(gripper.OpenGripper())

    # Move Robot up:
    robot.move_ee_up_down(env, delta_z = height, speed = 1, real_robot = True)


env.hold