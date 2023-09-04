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
        
        # Create an object of OnRobotROutpu():
        self.command = OnRobotRGOutput()

        self.command_move = OnRobotRGOutput()
    
    ## - OpenGripper Function:
    def OpenGripper(self,force=400):
        
        # Define a message:
        self.command.rGFR = force
        self.command.rGWD = 1100
        self.command.rCTR = 16

        return self.command
    
    ## - CloseGripper Function:
    def CloseGripper(self, force=400):
        # Define a message:
        self.command.rGFR = force
        self.command.rGWD = 0
        self.command.rCTR = 16

        return self.command

    ## - MoveGripperToPosition Function:
    def MoveGripperToPosition(self, force=400, position=1100):
        
        # Define a message:
        self.command_move.rGFR = force
        self.command_move.rGWD = max(0, position)
        self.command_move.rCTR = 16

        return self.command_move
        
        