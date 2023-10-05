#!/usr/bin/env python3

# -- Import necessary Python packages
import rospy, rosbag, os, sys, threading
from sensor_msgs.msg import Image

# Get the current script directory
script_dir = os.path.dirname(os.path.abspath(__file__))
# Append the parent directory of the script directory to the Python path
sys.path.append(os.path.join(script_dir, ".."))

# from Classes.GripperBaseClass import Gripper
from Classes.UR3e import*
from Classes.UR3eWithRealsense import UR3eWithRealSense

# Setup the ROS Node:
rospy.init_node('robot_node')

# Setup the Robot:
robot = UR3e()

# Setup the Swift Environment:
env = swift.Swift()
env.launch(realtime=True)
env._send_socket
env.add(robot.model)

# Set the initial position of the robot:
home_position = [-pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0]
robot.model.q = [-pi/2, -pi/2, -pi/2, -pi/2, pi/2, 0]

# Generate waypoints
waypoints = robot.generate_path(delta_x=-0.2, steps=50)

# Create a ROS bag
bag = rosbag.Bag('my_images.bag', 'w')

# Define a condition variable to control the image capturing thread
capture_condition = threading.Condition()

def capture_image(bag):
    with capture_condition:
        # Wait for the signal to capture the image
        capture_condition.wait()
        
        # Get the image message from the topic
        image_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=5)
        
        # Record the image message to the ROS bag
        bag.write('/camera/color/image_raw', image_msg)

# Create a thread to capture image without delaying robot movement
image_thread = threading.Thread(target=capture_image, args=(bag,))
image_thread.start()

# Generate waypoints
waypoints = robot.generate_path(delta_x=-0.2, steps=50)

# Begin moving and recording:
for i in range(len(waypoints) - 1): 
    start_point = waypoints[i]
    end_point = waypoints[i + 1]
    path = robot.perform_rmrc_2_points(start_point, end_point, num_waypoints=2)

    # Move the robot one step:
    robot.move_simulation_robot(path=path, env=env)
    # robot.send_trajectory_to_client(path, speed=1)

    # Signal the image thread to capture an image
    with capture_condition:
        capture_condition.notify()
    print(i)

# Ensure the image thread finishes before closing the bag
image_thread.join()

# Close the bag
bag.close()