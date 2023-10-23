#!/usr/bin/env python

import rospy
from control_msgs.msg import JointJog
from std_msgs.msg import Header

rospy.init_node('test_node', anonymous=True)

pub = rospy.Publisher('/servo_server/delta_joint_cmds', JointJog, queue_size=10)

# Create a new JointJog message
jog_cmd = JointJog()
jog_cmd.header = Header(stamp=rospy.Time.now())

# Set the joint names:
jog_cmd.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# Fill in the header:
jog_cmd.header.frame_id = "base_link"

jog_cmd.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

rate = rospy.Rate(10)  # 10Hz

pub.publish(jog_cmd)

rospy.spin()