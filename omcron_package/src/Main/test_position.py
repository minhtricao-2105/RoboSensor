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

robot.update_robot_position(q_home)

point_1 = np.array([-0.0137, 0.0255, 0.285, 1])

point_1_ee = np.array([point_1[0] - 0.0329 , point_1[1] + 0.0405, point_1[2] - 0.0632, 1])

print(point_1)

# Convert point_1_transformed to a 4x4 matrix
point_1_matrix = np.eye(4)  # Create a 4x4 identity matrix
point_1_matrix[:, -1] = point_1

# Generate cube:
cube_1 = collisionObj.Cuboid(scale=[0.06,0.06,0.06], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
cube_1.T = transl(0.0175, 0.31, 0)
env.add(cube_1)

print("Point 1: ", point_1[0])
print("Point 2: ", point_1[1])

robot.move_ee_up_down(env, delta_x=-point_1_ee [0], delta_y=point_1_ee [1], delta_z= -point_1_ee [2],real_robot=False)
env.hold()


