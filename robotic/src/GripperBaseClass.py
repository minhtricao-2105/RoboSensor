from onrobot_rg_control.msg import OnRobotRGOutput
import sys
import copy
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import math
import numpy as np

class Gripper:
    def __init__(self, node_name):
        
        # Define a ROS NODE
        self.node = rospy.init_node(node_name, anonymous=True)

        # Define a Publisher:
        self.pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)

        # Create an object of OnRobotROutpu():
        self.command = OnRobotRGOutput()
    
    ## - OpenGripper Function:
    def OpenGripper(self,force=400):
        
        # Define a message:
        self.command.rGFR = force
        self.command.rGWD = 1100
        self.command.rCTR = 16

        # Publish a message:
        self.pub(self.command)

        # Small delay for the gripper to open:
        rospy.sleep(0.3)
    
    ## - CloseGripper Function:
    def CloseGripper(self, force=400):
        # Define a message:
        self.command.rGFR = force
        self.command.rGWD = 0
        self.command.rCTR = 16

        # Publish a message:
        self.pub(self.command)

        # Small delay for the gripper to close:
        rospy.sleep(0.3)

    ## - MoveGripperToPosition Function:
    def MoveGripperToPosition(self, force=400, position=1100):

        # Define a message:
        self.command_move.rGFR = force
        self.command_move.rGWD = max(0, position)
        self.command_move.rCTR = 16

        # Publish a message:
        self.pub(self.command)

        # Small delay for the gripper to close:
        rospy.sleep(0.3)
        
        