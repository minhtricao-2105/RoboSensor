#!/usr/bin/env python3
# -- Import necessary Python packages
import rospy, rosbag, os, sys, threading
from sensor_msgs.msg import Image

import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def moveit_ur3e_joint_control():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('moveit_ur3e_joint_control', anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

    # IMPORTANT: Replace the following joint names with the joints of your simulated UR3e robot.
    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

    joint_goal = move_group.get_current_joint_values()
    # Define your desired joint positions here, for example:
    q_home = [-56.0, -93.8, -41.05, -135.07, 90.0, 34.0]
    q_home = [math.radians(angle) for angle in q_home]

    joint_goal = q_home
    joint_goal[5] = joint_goal[5] + 0.5
    print(q_home)
    print(joint_goal)

    # IMPORTANT: Set the joint state target using the joint names and the corresponding joint goals.
    joint_state_target = dict(zip(joint_names, joint_goal))
    move_group.set_joint_value_target(joint_state_target)

    move_group.go(joint_goal, wait=True)
    move_group.stop()

    moveit_commander.roscpp_shutdown()

def send_joint_trajectory():
    rospy.init_node('joint_trajectory_action_client')
    client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    client.wait_for_server()

    trajectory = JointTrajectory()
    trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

    point = JointTrajectoryPoint()
    # Set your desired joint positions here
    point.positions = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
    point.time_from_start = rospy.Duration(5)  # Adjust the duration as needed

    trajectory.points.append(point)

    goal = FollowJointTrajectoryGoal()
    goal.trajectory = trajectory

    client.send_goal(goal)
    client.wait_for_result()

if __name__=='__main__':
    try:
        send_joint_trajectory()
    except rospy.ROSInterruptException:
        pass
