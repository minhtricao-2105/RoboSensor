# Import Library Needed:
import numpy as np
import rospy, time, actionlib, moveit_msgs.msg, moveit_commander, math, sys, swift
import roboticstoolbox as rtb 

# Python Message via ROS:
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from spatialmath import SE3
from sensor_msgs.msg import JointState
from math import pi

class RobotBaseClass:
    
    def __init__(self):
            
        # Define a Subcriber:
        self.subscriber = rospy.Subscriber('/joint_states', JointState, self.getpos)

        # Create an object of JointTrajectory()
        self.joint_traj = JointTrajectory()

        # Create a FollowJointTrajectoryGoal message:
        self.goal = FollowJointTrajectoryGoal()
        
        # Set up Action Client when creating an object
        self.set_up_action_client()

        # Set up the Client (change the topic if needed:)
        # self.client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        self.client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # Create a Time Flag when creating an object:
        self.start_time = time.perf_counter()

        # Data Member of the class
        self.currentQ = []

        self.model = rtb.models.UR3()

    ##---- CallBack Function:
    def getpos(self,msg):
        self.currentQ = msg.position
        
    ##---- set_up_action_client function:

    def set_up_action_client(self):
        
        # Set the joint names:
        self.joint_traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

        # Fill in the header:
        self.joint_traj.header.frame_id = "base_link"

        # Set up the joint names:
        self.goal.trajectory.joint_names = self.joint_traj.joint_names

        # Set up Sequence:
        self.goal.trajectory.header.seq = 1

        # Set up Time Stamp
        self.goal.trajectory.header.stamp = rospy.Time.now()

        # Set up the tolerance:
        self.goal.goal_time_tolerance = rospy.Duration.from_sec(0.05)

    ##---- convert_trajectory_to_rosmsg function:

    def send_trajectory_to_client(self, path,  speed=1):
        
        # Clear the goal beforing adding new point:
        self.goal.trajectory.points.clear()

        # Set up the clock:
        end_time = time.perf_counter()

        # Calculate the excution time:
        execution_time = end_time - self.start_time

        for i in range(len(path)):
            point = JointTrajectoryPoint()

            point.positions = path[i]

            point.time_from_start = rospy.Duration.from_sec((i+1)*(speed/len(path))) + rospy.Duration.from_sec(execution_time + 1)

            self.goal.trajectory.points.append(point)
        
        # Send the goal to the client:
        self.client.send_goal(self.goal)

        self.client.wait_for_result()

        result = self.client.get_result()

        
    def combine_trajectories(self, qlist):
        
        total_q = []
        
        for trajectory in qlist:
            for q in trajectory.q:
                total_q.append(q)

        return total_q

    def set_up_moveIt(self,maxVelocity):
        
        # Initialize the moveit_commander and rospy nodes
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the MoveIt planning scene, robot commander, and arm group
        scene = moveit_commander.PlanningSceneInterface()
        robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("manipulator")

        # Set the reference frame and end effector link
        self.arm.set_pose_reference_frame("base_link")
        self.arm.set_end_effector_link("ee_link")

        # Set the maximum velocity scaling factor
        self.arm.set_max_velocity_scaling_factor(maxVelocity)

        # Get the current joint values
        current_joint_values = self.arm.get_current_joint_values()

    def move_jtraj(self, q, q0, steps = 50, speed = 1):
        
        # Generate a path:
        path = rtb.jtraj(q, q0, steps)

        # Send to messange to the robot
        self.send_trajectory_to_client(path.q, speed)

        

    

        
        

    
        



        


        