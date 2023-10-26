import rospy, swift
import numpy as np
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb 

class Position:
    def __init__(self):
        self.position = JointState()
        self.sub = rospy.Subscriber('/joint_states', JointState, self.callback)
        # Set up the robot model:
        self.model = rtb.models.UR3()

    def callback(self, msg):
        self.position = msg
        self.model.q = np.array(self.position.position)
        swap = self.model.q[0]
        self.model.q[0] = self.model.q[2]
        self.model.q[2] = swap
        rospy.loginfo("Robot joint state: %s", self.model.q)

if __name__ == "__main__":
    rospy.init_node('position_node')
    position = Position()
    rospy.spin()