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
from OmcronBaseClass.Gripper import*
from OmcronBaseClass.Mission import*

# Setup the ROS Node:
rospy.init_node('robot_node')

# Setup the Robot:
robot = UR3e()

# Setup the Swift Environment:
env = swift.Swift()
env.launch(realtime=True)
env._send_socket
env.add(robot.model)
env.add(robot._cam)
env.add(robot._gripper)

# Setup the initial position of the robot:
q_home = [-56.0, -93.8, -41.05, -135.07, 90.0, 34.0]
q_home = [math.radians(angle) for angle in q_home]

# Using the UR3e robot, move the robot to the home position:
try:
    robot.update_robot_position(q_home)
    robot.set_up_moveIt(0.1)
    robot.arm.go(q_home, wait=True)
    print("Robot is in the home position.")
except:
    print("No Moveit Driver is running or something went wrong...")

# Move to the other side (rotate 180)
desiredQ = robot.model.q.copy()
desiredQ[0] = desiredQ[0] + pi

robot.move_jtraj(robot.model.q, desiredQ, env, 50, real_robot=True)
q_home_drop = [102.93, -131.0, -42.63, -98.49, 89.91, 11.40]
q_home_drop = [math.radians(angle) for angle in q_home_drop]
robot.move_jtraj(robot.model.q, q_home_drop, env, 50, real_robot=True)

robot.move_ee_up_down(env, delta_x=0, delta_y=0, delta_z= -0.18,real_robot=True)
# robot.move_ee_up_down(env, delta_x=0, delta_y=-0.16, delta_z= 0,real_robot=True)
# robot.move_ee_up_down(env, delta_x=-0.1, delta_y=0, delta_z= 0,real_robot=True)
# robot.move_ee_up_down(env, delta_x=0, delta_y=0, delta_z= -0.29,real_robot=True)
# robot.send_velocity_trajectory([0,0,0,0,0,0.3], 10)

