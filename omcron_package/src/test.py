#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from omcron_package.srv import *
from math import pi

# Create a service client
client = rospy.ServiceProxy('tm_driver/set_positions', SetPosition)

# Wait for the service to be available
rospy.wait_for_service('tm_driver/set_positions')

# Define the request
srv_request = SetPositionRequest()

# Setting Request Data:
srv_request.motion_type = 1
srv_request.positions = [0, 0, 0, 0, 0, 0]
srv_request.velocity = 0.4
srv_request.acc_time = 0.2
srv_request.blend_percentage = False
srv_request.fine_goal = False

# Call the service
try:
    srv_response = client(srv_request)
    if srv_response.ok:
        rospy.loginfo("SetPositions to robot")
    else:
        rospy.logwarn("SetPositions to robot, but response not yet ok")
except rospy.ServiceException as e:
    rospy.logerr("Error SetPositions to robot: %s", str(e))