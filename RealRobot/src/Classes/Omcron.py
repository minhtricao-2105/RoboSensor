import os
import numpy as np
import rospy, time, actionlib, moveit_msgs.msg, moveit_commander, math, sys, swift
import roboticstoolbox as rtb 
import copy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from spatialmath import SE3
from sensor_msgs.msg import JointState
from math import pi

# Setup the ROS Node:
rospy.init_node('robot_node')

# Setup currentQ
currentQ = None

# ##---- CallBack Function:
# def getpos(msg):
#     currentQ = msg.position

# subscriber = rospy.Subscriber('/joint_states', JointState, getpos)

# Create an object of JointTrajectory()
joint_traj = JointTrajectory()

# Create a FollowJointTrajectoryGoal message:
goal = FollowJointTrajectoryGoal()

# Create a FollowJointTrajectoryAction client:
joint_traj.joint_names = ['shoulder_1_joint', 'shoulder_2_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Set up the joint trajectory:
joint_traj.header.frame_id = "base_link"
goal.trajectory.joint_names = joint_traj.joint_names
goal.trajectory.header.seq = 1
goal.trajectory.header.stamp = rospy.Time.now()
goal.goal_time_tolerance = rospy.Duration.from_sec(0.05)

# Setup the Client:
client = actionlib.SimpleActionClient('/joint_group_position_controller/command', FollowJointTrajectoryAction)

start_time = time.perf_counter()

goal.trajectory.points.clear()

# Set up the clock:
end_time = time.perf_counter()

# Calculate the excution time:
execution_time = end_time - start_time

desiredQ = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

print("The desired joint angles are: ", desiredQ)

# Build the trajectory:
point = JointTrajectoryPoint()

point.positions = desiredQ

point.time_from_start = rospy.Duration.from_sec(execution_time)

goal.trajectory.points.append(point)

# Send the goal to the client:
client.send_goal(goal)

client.wait_for_result()

result = client.get_result()




