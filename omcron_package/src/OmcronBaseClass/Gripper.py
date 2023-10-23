from onrobot_rg_control.msg import OnRobotRGOutput
import sys
import copy
import rospy
import geometry_msgs.msg
from std_msgs.msg import String
import math
import numpy as np

class Gripper:
    def __init__(self):
        
        # Create an object of OnRobotROutput():
        self.command = OnRobotRGOutput()

        # Setup the publisher
        pub = rospy.Publisher('OnRobotRGOutput', OnRobotRGOutput, queue_size=1)
    
    ## - OpenGripper Function:
    def OpenGripper(self,force=400):
        # Define a message:
        self.command.rGFR = force
        self.command.rGWD = 1100
        self.command.rCTR = 16
        self.pub.publish(self.command)

        return self.command
    
    ## - CloseGripper Function:
    def CloseGripper(self, force=400):
        # Define a message:
        self.command.rGFR = force
        self.command.rGWD = 0
        self.command.rCTR = 16
        self.pub.publish(self.command)

        return self.command

    ## - MoveGripperToPosition Function:
    def MoveGripperToPosition(self, force=400, position=1100):
        # Define a message:
        self.command.rGFR = force
        self.command.rGWD = max(0, position)
        self.command.rCTR = 16
        self.pub.publish(self.command)

        return self.command