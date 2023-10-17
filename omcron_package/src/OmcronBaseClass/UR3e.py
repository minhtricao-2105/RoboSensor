# Import Library Needed:
import numpy as np
import rospy, time, actionlib, moveit_msgs.msg, moveit_commander, math, sys, swift
import roboticstoolbox as rtb 
import copy
import spatialgeometry.geom as collisionObj

# Python Message via ROS:
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from spatialmath import SE3
from sensor_msgs.msg import JointState
from math import pi
from scipy.spatial.transform import Rotation as R
from spatialmath.base import *

class UR3e:
    
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
        self.client = actionlib.SimpleActionClient('scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # self.client = actionlib.SimpleActionClient('eff_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        
        # Create a Time Flag when creating an object:
        self.start_time = time.perf_counter()

        # Data Member of the class
        self.currentQ = []

        # Set up the robot model:
        self.model = rtb.models.UR3()

        # Setup Gripper:
        self._gripper_path = "/home/minhtricao/git/UTS_RS2_VNG_Team/RTB-P Test Files/SomeApplications/CAMGRIPPER.STL"
        self._gripper = collisionObj.Mesh(filename=self._gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [1.0,0.0,0.0,1])
        self._TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)

        # Setup the camera:
        self._cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
        self._TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) # cam pose in ee frame

        self._cam_move(self._cam, self.model, self._TCR)
        self._cam_move(self._gripper, self.model, self._TGR)

    ##---- CallBack Function:
    def getpos(self,msg):
        self.currentQ = msg.position
    
    ## --- Move the Camera and the Gripper to the end-effector:
    def _cam_move(self, cam, robot, T):
        cam.T = robot.fkine(robot.q)*T

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

    ##---- combine_trajectories function:

    def combine_trajectories(self, qlist):
        
        total_q = []
        
        for trajectory in qlist:
            for q in trajectory.q:
                total_q.append(q)

        return total_q

    ##---- set_up_moveIt function:

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

    ##---- move_jtraj function:

    def move_jtraj(self, q0, q, env, steps = 50, speed = 1, real_robot = False):
        
        # Generate a path:
        path = rtb.jtraj(q0, q, steps)

        # Send to messange to the robot
        if real_robot:
            self.send_trajectory_to_client(path.q, speed)
            
            # update to robot.q
            self.model.q = path.q[-1, :]
        else:
            self.move_simulation_robot(path.q, env = env)

        return path

    ##---- move_simulation_robot function:

    def move_simulation_robot(self, path, env, dt = 0.05):
        # Simulation the Robot in the Swift Environment:
        for q in path:
            self.model.q = q
            self._cam_move(self._cam, self.model, self._TCR)
            self._cam_move(self._gripper, self.model, self._TGR)    
            env.step(dt)

    ##---- perform_rmrc_2_points function:
    def perform_rmrc_2_points(self, point1, point2, num_waypoints = 100):

        # Initialize the waypoints matrix:
        waypoints = np.zeros((num_waypoints, 3))

        # Linear interpolation between the two points:
        for i in range(num_waypoints):
            t = i / (num_waypoints - 1) # Interpolation parameter [0, 1]
            waypoints[i, :] = (1 - t)*point1 +t*point2
        
        # Create a matrix of joint angles:
        q_matrix = np.empty((num_waypoints, self.model.n))
        q_matrix[0, :] = self.model.q
        
        # The delta T between each steps:
        deltaT = 0.05
        
        # Perform RMRC:
        for i in range(num_waypoints - 1):
            # Get the forward transformation at current joint states:
            T = self.model.fkine(q_matrix[i,:]).A

            # Get Position Error:
            deltaX = waypoints[i+1, :] - T[0:3, 3]

            # Get the Linear Velocity:
            linearVelocity = (1/deltaT) * deltaX

            # Get the angular velocity:
            angularVelocity = np.array([0, 0, 0])

            # Calculate the end-effector velocity to reach next waypoint:
            xDot = np.concatenate((linearVelocity, angularVelocity), axis=None)

            # Find the Jacobian Matrix at the current pose:
            J = self.model.jacob0(q_matrix[i, :])

            # Solve velocitites by using pinv:
            q_dot = np.linalg.pinv(J) @ xDot

            # Update next Joint State:
            q_matrix[i+1, :] = q_matrix[i, :] + deltaT*q_dot

        # Update to robot.q
        self.model.q = q_matrix[-1, :]

        return q_matrix
    
    ##---- move_ee_up_down function:
    def move_ee_up_down(self, env, delta_x = 0, delta_y = 0, delta_z = 0, speed = 1, real_robot = False):
        # Get the end-effector pose at this position:
        ee_tr = self.model.fkine(self.model.q).A

        # Get the point:
        point_1 = np.array([ee_tr[0,3], ee_tr[1,3], ee_tr[2,3]])

        # Get the desire point to move the end-effector down:
        point_2 = np.array([ee_tr[0,3] + delta_x, ee_tr[1,3] + delta_y, ee_tr[2,3]+ delta_z])

        # Perform RMRC:
        path = self.perform_rmrc_2_points(point_1, point_2)

        # Simulation the movement:
        if real_robot:
            self.send_trajectory_to_client(path, speed)
        else:
            self.move_simulation_robot(path, env = env)

    ##---- get_joint_list function:

    def get_joint_list(self) -> list:

        joint_list = [] # List of joints

        j = 0
        for link in self.model.links:
            if link.isjoint:
                joint_list.append(j)
            j += 1

        return joint_list

    ##---- get_link_transform function:

    def get_link_transform(self, q) -> list:
        transform_list = [] # Tranforms array of link
        joint_list = self.get_joint_list() # List of joints as link index

        for i in joint_list:
            transform_list.append(self.model.fkine(q,end = self.model.links[i].name)) 
        return transform_list    
    
    ##---- is_touch_ground function:

    def is_touch_ground(self,q)->bool:
        
        # Get the link transform of each link:
        transform_list = self.get_link_transform(q)

        # Calculate Height of each link relative to the base of the robot:
        height = [T.A[2,3] for T in transform_list]

        # Check the codition of touching ground:
        if min(height) > transform_list[0].A[2,3]: # at position 0 is the robot base
            return False
        else: 
            return True

    def solve_elbow_up_ikine(self, desired_T, q_guess, env, speed = 1, real_robot = False):

        # Get the current joint angles:
        currentQ = q_guess

        # Get the parameters of variations:
        delta_rot = np.pi/8

        # Define the variations:
        variations = np.array([
            [delta_rot, 0, 0, 0, 0, 0],
            [-delta_rot, 0, 0, 0, 0, 0],
            [0, delta_rot, 0, 0, 0, 0],
            [0, -delta_rot, 0, 0, 0, 0],
            [0, 0, delta_rot, 0, 0, 0],
            [0, 0, -delta_rot, 0, 0, 0]
        ])

        # Define the initial guess:
        guesses = currentQ + variations

        solutions = []
        
        # Loop through all the guesses for solving the inverse kinematics:
        for i in range(guesses.shape[0]):
            sol = self.model.ikine_LM(desired_T, q0=guesses[i, :], ilimit=100, method='pseudoinverse').q
            if sol is not None:
                solutions.append(sol)

        # Check if there is any solution:
        if not solutions:
            raise ValueError('No IK solutions found')
        
        # Convert the solutions to numpy array:
        solutions = np.array(solutions)
        
        # Check if there is any solution with elbow up:
        elbowUpSolutions = solutions[solutions[:, 2] > 0]
        otherSolutions = solutions[solutions[:, 2] <= 0]

        # If there is any solution with elbow up, choose the one with the smallest distance to the current joint angles:
        if elbowUpSolutions.any():
            distances = np.sum((elbowUpSolutions - currentQ)**2, axis=1)
            idx = np.argmin(distances)
            q_final = elbowUpSolutions[idx, :]
        else:
            distances = np.sum((otherSolutions - currentQ)**2, axis=1)
            idx = np.argmin(distances)
            q_final = otherSolutions[idx, :]
        
        # Generate the path using jtraj
        path = rtb.jtraj(self.model.q, q_final, 50)

        # Send to messange to the robot
        if real_robot:
            self.send_trajectory_to_client(path.q, speed)
            
            # update to robot.q
            self.model.q = path.q[-1, :]
        else:
            self.move_simulation_robot(path.q, env = env)

        return path
    

    def rotate_ee(self, env, degree = 90, speed = 1, real_robot = False):

        desired_q = copy.deepcopy(self.model.q + np.array([0, 0, 0, 0, 0, np.deg2rad(degree)]))

        print(desired_q)
        # Create a jtraj:
        path = rtb.jtraj(self.model.q, desired_q, 50)

        # Send to messange to the robot
        if real_robot:
            self.send_trajectory_to_client(path.q, speed)
            
            # update to robot.q
            self.model.q = path.q[-1, :]
        else:
            self.move_simulation_robot(path.q, env = env)

        return path
    

    def generate_path(self, delta_x = 0, delta_y = 0, delta_z = 0, steps = 25):
        
        # Get the end-effector pose at this position:
        ee_tr = self.model.fkine(self.model.q).A

        # Get the point:
        start_point = np.array([ee_tr[0,3], ee_tr[1,3], ee_tr[2,3]])

        # Get the desire point to move the end-effector down:
        end_point = np.array([ee_tr[0,3] + delta_x, ee_tr[1,3] + delta_y, ee_tr[2,3]+ delta_z]) 

        # Generate path waypoints:
        waypoints = np.zeros((steps, 3))

        for i in range(steps):
            # Interpolation parameter [0, 1]
            t = i / (steps - 1)

            # Linear interpolation between the two points:
            waypoints[i, :] = (1 - t)*start_point +t*end_point
        
        return waypoints


    
        



        


        