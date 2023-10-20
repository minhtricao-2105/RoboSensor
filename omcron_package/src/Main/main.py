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

# Setup the Camera:
camera = Camera()

# Setup the Gripper:
gripper = Gripper()

rospy.sleep(2)

# Begin the main loop of the program:
if __name__ == '__main__':
    print('Program is running...')

    detected_objects_blue = []

    while not detected_objects_blue:
        detected_objects_blue = camera.detect_object('blue')
    
    point = []
    
    for K in detected_objects_blue:
        # Convert to 3D point
        x, y, depth, label = K
        
        point_temp = camera.project_2D_to_3D(x, y, depth)
        point.append(point_temp)

        # print(T)
        # print(point)
    
    point_1_ee = point[0]
    point_1_ee[2] = point_1_ee[2] - 0.18
    
    point_2_ee = point[1]
    point_2_ee[2] = point_2_ee[2] - 0.18

    print(point_1_ee)
    


    robot.move_ee_up_down(env, delta_x=-point_1_ee [0], delta_y=point_1_ee [1], delta_z= -point_1_ee [2],real_robot=True)

    robot.move_jtraj(robot.model.q, q_home, env, 50, real_robot=True)

    robot.move_ee_up_down(env, delta_x=-point_2_ee [0], delta_y=point_2_ee [1], delta_z= -point_2_ee [2],real_robot=True)

