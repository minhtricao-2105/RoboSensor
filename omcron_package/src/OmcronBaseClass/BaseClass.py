# Import Library Needed:
import numpy as np
import roboticstoolbox as rtb
import rospy, copy

# Python Message Types:
from std_msgs.msg import String
from omcron_package.srv import *


class OmcronBaseClass:

    def __init__(self):

        # Create a service client inside the class when an object is created
        self.client = rospy.ServiceProxy('tm_driver/set_positions', SetPosition)

        # Wait for the service to be available
        rospy.wait_for_service('tm_driver/set_positions')

        # Define the request
        self.srv_request = SetPositionRequest()

        # Define a simulated robot inside the class:
        self.model = []

        # Create a simulated robot inside the class:
        self._create_DH()
        
    def _create_DH(self):

        """
        Create robot's standard DH model
        """
        # deg = np.pi / 180
        mm = 1e-3

        # kinematic parameters
        a = np.r_[0, -329, -311.5, 0, 0, 0] * mm
        d = np.r_[146, 0, 0, 129.7, 106, 113.2] * mm
        alpha = [np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0]
        qlim = [[-2*np.pi, 2*np.pi] for _ in range(7)]

        # offset to have the dh from toolbox match with the actual pos
        offset = [0, -np.pi/2, 0, -np.pi/2, 0, 0]
        links = []
        for j in range(6):
            link = rtb.RevoluteDH(
                d=d[j], a=a[j], alpha=alpha[j], offset=offset[j], qlim=qlim[j])
            links.append(link)

        # Create a simulated robot inside the class:
        self.model = rtb.DHRobot(links, name="MyRobot")
    
    def send_position_to_robot(self, type = 1, position = [], velocity = 0.4, acc_time = 0.2, blend_percentage = False, fine_goal = False):

        # Setting Request Data:
        self.srv_request.motion_type = type
        self.srv_request.positions = position
        self.srv_request.velocity = velocity
        self.srv_request.acc_time = acc_time
        self.srv_request.blend_percentage = blend_percentage
        self.srv_request.fine_goal = fine_goal

        # Call the service
        try:
            srv_response = self.client(self.srv_request)
            if srv_response.ok:
                rospy.loginfo("SetPositions to robot")
            else:
                rospy.logwarn("SetPositions to robot, but response not yet ok")
        except rospy.ServiceException as e:
            rospy.logerr("Error SetPositions to robot: %s", str(e))

    def get_robot_position(self):
        return self.model.q
    
    def send_trajectory_to_robot(self, path, velocity = 0.4, acc_time = 0.2, blend_percentage = False, fine_goal = False):

        for q in path.q:
            self.send_position_to_robot(type = 1, position = q, velocity = velocity, acc_time = acc_time, blend_percentage = blend_percentage, fine_goal = fine_goal)
            rospy.sleep(0.3)
