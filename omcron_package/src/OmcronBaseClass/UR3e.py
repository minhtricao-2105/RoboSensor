# Import Library Needed:
import numpy as np
import rospy, time, actionlib, moveit_msgs.msg, moveit_commander, math, sys, swift
import roboticstoolbox as rtb 
import copy, rospkg
import spatialgeometry.geom as collisionObj

# Python Message via ROS:
import sys, rospy, actionlib
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import SwitchControllerRequest, SwitchController
from controller_manager_msgs.srv import LoadControllerRequest, LoadController
from controller_manager_msgs.srv import ListControllers, ListControllersRequest
import geometry_msgs.msg as geometry_msgs
from cartesian_control_msgs.msg import (
    FollowCartesianTrajectoryAction,
    FollowCartesianTrajectoryGoal,
    CartesianTrajectoryPoint,
)

# Other Library:
from spatialmath import SE3
from sensor_msgs.msg import JointState
from math import pi
from scipy.spatial.transform import Rotation as R
from spatialmath.base import *

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

JOINT_TRAJECTORY_CONTROLLERS = [
    "scaled_pos_joint_traj_controller",
    "scaled_vel_joint_traj_controller",
    "pos_joint_traj_controller",
    "vel_joint_traj_controller",
    "forward_joint_traj_controller",
]

CARTESIAN_TRAJECTORY_CONTROLLERS = [
    "pose_based_cartesian_traj_controller",
    "joint_based_cartesian_traj_controller",
    "forward_cartesian_traj_controller",
    "twist_controller",
]

CONFLICTING_CONTROLLERS = ["joint_group_vel_controller", "twist_controller"]

class UR3e:
    
    def __init__(self):

        timeout = rospy.Duration(5)
        self.switch_srv = rospy.ServiceProxy(
            "controller_manager/switch_controller", SwitchController
        )
        self.load_srv = rospy.ServiceProxy("controller_manager/load_controller", LoadController)
        self.list_srv = rospy.ServiceProxy("controller_manager/list_controllers", ListControllers)
        try:
            self.switch_srv.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            rospy.logerr("Could not reach controller switch service. Msg: {}".format(err))
            sys.exit(-1)

        # Controller:
        self.joint_trajectory_controller = JOINT_TRAJECTORY_CONTROLLERS[0]
        self.cartesian_trajectory_controller = CARTESIAN_TRAJECTORY_CONTROLLERS[3]
        
        # Publisher:
        self.pub = rospy.Publisher('/twist_controller/command', Twist, queue_size=10)
        self.command_vel = Twist()

        # Subcriber:
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.joint_states = JointState()

        # Action Client:
        self.trajectory_client = actionlib.SimpleActionClient(
            "{}/follow_joint_trajectory".format(self.joint_trajectory_controller),
            FollowJointTrajectoryAction,
        )

        # Switch to joint trajectory controller for Moveit:
        self.switch_controller(self.joint_trajectory_controller)

        # Set up the robot model:
        self.model = rtb.models.UR3()

        # Get the package path:
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('omcron_package')
        
        # Setup Gripper:
        self._gripper_path = package_path + '/GripperModel/CAMGRIPPER.STL'

        self._gripper = collisionObj.Mesh(filename=self._gripper_path,pose = SE3(0,0,0),scale=[0.001, 0.001, 0.001],color = [1.0,0.0,0.0,1])
        self._TGR = SE3.Rx(pi)*SE3(0,-0.105,-0.175)

        # Setup the camera:
        self._cam = collisionObj.Cuboid(scale=[0.03,0.06,0.015], pose = SE3(0,0,0), color = [0.5,0.5,0.1,1])
        self._TCR = SE3(0.085,0,0.09)*SE3.Ry(pi/2) # cam pose in ee frame

        self._cam_move(self._cam, self.model, self._TCR)
        self._cam_move(self._gripper, self.model, self._TGR)
    
    # --- Joint States Callback:
    def joint_states_callback(self, msg):
        self.joint_states = msg

    # --- Switch Controller:
    def switch_controller(self, target_controller):
        """Activates the desired controller and stops all others from the predefined list above"""
        other_controllers = (
            JOINT_TRAJECTORY_CONTROLLERS
            + CARTESIAN_TRAJECTORY_CONTROLLERS
            + CONFLICTING_CONTROLLERS
        )

        other_controllers.remove(target_controller)

        srv = ListControllersRequest()
        response = self.list_srv(srv)
        for controller in response.controller:
            if controller.name == target_controller and controller.state == "running":
                return

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_srv(srv)

        srv = SwitchControllerRequest()
        srv.stop_controllers = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_srv(srv)

    ## --- Update Postion to simulation robot:
    def update_robot_position(self, q):
        self.model.q = q
        self._cam_move(self._cam, self.model, self._TCR)
        self._cam_move(self._gripper, self.model, self._TGR)

    ## --- Move the Camera and the Gripper to the end-effector:
    def _cam_move(self, cam, robot, T):
        cam.T = robot.fkine(robot.q)*T

    ## --- Send Joint Trajectory:
    def send_joint_trajectory(self, path, speed = 0.3):
        """Creates a trajectory and sends it using the selected action server
        Args:
        - path: A list of joint configurations. Each configuration is a list of joint angles.
        """

        # make sure the correct controller is loaded and activated
        self.switch_controller(self.joint_trajectory_controller)


        # Wait for action server to be ready
        timeout = rospy.Duration(5)
        if not self.trajectory_client.wait_for_server(timeout):
            rospy.logerr("Could not reach controller action server.")
            sys.exit(-1)

        # Create and fill trajectory goal
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = JOINT_NAMES

        # Assuming each point in the path should be reached in a constant time interval
        time_interval = speed  # Adjust as necessary
        for i in range(len(path)):
            point = JointTrajectoryPoint()
            point.positions = path[i]
            point.time_from_start = rospy.Duration((i + 1) * time_interval)
            goal.trajectory.points.append(point)

        rospy.loginfo("Executing trajectory using the {}".format(self.joint_trajectory_controller))

        self.trajectory_client.send_goal(goal)
        self.trajectory_client.wait_for_result()

        result = self.trajectory_client.get_result()
        rospy.loginfo("Trajectory execution finished in state {}".format(result.error_code))

    ##---- combine_trajectories function:
    def combine_trajectories(self, qlist):
        """
        Combines a list of trajectories into a single trajectory
        Args:
            - qlist: A list of trajectories. Each trajectory is a list of joint configurations.
        """
        total_q = []
        
        for trajectory in qlist:
            for q in trajectory.q:
                total_q.append(q)

        return total_q

    ##---- set_up_moveIt function:
    def set_up_moveIt(self,maxVelocity):
        """
        Set up the MoveIt planning scene, robot commander, and arm group
        """
        
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
    def move_jtraj(self, q0, q, env, steps = 50, speed = 0.3, real_robot = False):
        """
        Move the robot from q0 to q using jtraj

        Args:
            - q0: The initial joint configuration
            - q: The final joint configuration
            - steps: The number of steps to take between q0 and q
            - speed: The speed at which to execute the trajectory
            - real_robot: If true, send the trajectory to the real robot. If false, send to the simulation
        """
        # Generate a path:
        path = rtb.jtraj(q0, q, steps)

        # Send to messange to the robot
        if real_robot:
            self.send_joint_trajectory(path.q, speed)
            
            # update to robot.q
            self.model.q = path.q[-1, :]
        else:
            self.move_simulation_robot(path.q, env = env)

        return path

    ##---- move_simulation_robot function:

    def move_simulation_robot(self, path, env, dt = 0.05):
        """
        Move the robot in the simulation environment

        Args:
            - path: A list of joint configurations. Each configuration is a list of joint angles.
            - env: The swift simulator environment
            - dt: The time between each step in the simulation
        """
        # Simulation the Robot in the Swift Environment:
        for q in path:
            self.model.q = q
            self._cam_move(self._cam, self.model, self._TCR)
            self._cam_move(self._gripper, self.model, self._TGR)    
            env.step(dt)

    ##---- perform_rmrc_2_points function:
    def perform_rmrc_2_points(self, point1, point2, num_waypoints = 100):
        """
        Move the robot from point1 to point2 using RMRC

        Args:
            - point1: The initial position of the end-effector
            - point2: The final position of the end-effector
            - num_waypoints: The number of waypoints to use in the trajectory
        """
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
    
    ## --- Send the Twist Message to the robot using Twist Controller:
    def send_velocity(self, x, y, z, roll, pitch, yaw):
        """
        Sending velocity using Twist Controller

        Args:
            - x: The linear velocity in x axis
            - y: The linear velocity in y axis
            - z: The linear velocity in z axis
            - roll: The angular velocity in roll axis
            - pitch: The angular velocity in pitch axis
            - yaw: The angular velocity in yaw axis
        """
        self.switch_controller(self.cartesian_trajectory_controller)
        self.command_vel.linear.x = x
        self.command_vel.linear.y = y
        self.command_vel.linear.z = z
        self.command_vel.angular.x = roll
        self.command_vel.angular.y = pitch
        self.command_vel.angular.z = yaw
        self.pub.publish(self.command_vel)

    ## --- Stop the Robot while being controlled by Twist Controller:
    def stop_robot(self):
        """
        Stop the robot while being controlled by Twist Controller
        """
        self.command_vel.linear.x = 0
        self.command_vel.linear.y = 0
        self.command_vel.linear.z = 0
        self.command_vel.angular.x = 0
        self.command_vel.angular.y = 0
        self.command_vel.angular.z = 0
        self.pub.publish(self.command_vel)

    ## --- move_ee_up_down function:
    def move_ee_up_down(self, env, delta_x = 0, delta_y = 0, delta_z = 0, speed = 0.05, real_robot = False):
        """
        Move the end-effector in the cartestian plane
        
        Args:
            - env: The swift simulator environment
            - delta_x: The distance to move in the x axis
            - delta_y: The distance to move in the y axis
            - delta_z: The distance to move in the z axis
            - speed: The speed at which to execute the trajectory
            - real_robot: If true, send the trajectory to the real robot. If false, send to the simulation
        """
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
            self.send_joint_trajectory(path, speed)
        else:
            self.move_simulation_robot(path, env = env)

    ## --- move_ee_velocity:
    def move_ee_velocity(self, delta_x = 0, delta_y = 0, delta_z = 0, speed = 0.05):
        """
        Move the end-effector in the cartestian plane by sending velocity (Note: this is just used for real robot)
        
        Args:
            - delta_x: The distance to move in the x axis
            - delta_y: The distance to move in the y axis
            - delta_z: The distance to move in the z axis
            - speed: The speed at which to execute the trajectory
        """
        # Get the end-effector pose at this position:
        ee_tr = self.model.fkine(self.model.q).A

        # Get the point:
        point_1 = np.array([ee_tr[0,3], ee_tr[1,3], ee_tr[2,3]])

        # Get the desire point to move the end-effector down:
        point_2 = np.array([ee_tr[0,3] + delta_x, ee_tr[1,3] + delta_y, ee_tr[2,3]+ delta_z])

        # Compute the direction vector:
        direction_vector = point_2 - point_1

        # Normalize the direction Vector to get a unit vector:
        unit_vector = direction_vector / np.linalg.norm(direction_vector)

        # Get the velocity vector:
        velocity_vector = speed * unit_vector

        # Calculate distance between two points:
        distance = np.linalg.norm(direction_vector)

        # Send Velocity to the Robot:
        while distance > 0.01:
            self.send_velocity(velocity_vector[0], velocity_vector[1], velocity_vector[2], 0, 0, 0)

            # Update robot.q:
            self.model.q = self.joint_states.position
            swap = self.model.q[0]
            self.model.q[0] = self.model.q[2]
            self.model.q[2] = swap

            # Get the end-effector pose at this position:
            ee_tr = self.model.fkine(self.model.q).A
            point_1 = np.array([ee_tr[0,3], ee_tr[1,3], ee_tr[2,3]])

            # Recalculate the distance:
            distance = np.linalg.norm(point_2 - point_1)
        
        # Stop the robot when reaching the desired point:
        self.stop_robot()

        # Update robot.q:
        self.model.q = self.joint_states.position

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

    def solve_elbow_up_ikine(self, desired_T, q_guess, env, speed = 0.5, real_robot = False):

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
            self.send_joint_trajectory(path.q, speed)
            
            # update to robot.q
            self.model.q = path.q[-1, :]
        else:
            self.move_simulation_robot(path.q, env = env)

        return path
    
    def rotate_ee(self, env, degree = 90, speed = 0.5, real_robot = False):
        """
        Rotate the end-effector in the cartestian plane

        Args:
            - env: The swift simulator environment
            - degree: The degree to rotate the end-effector
            - speed: The speed at which to execute the trajectory
            - real_robot: If true, send the trajectory to the real robot. If false, send to the simulation
        """
        desired_q = copy.deepcopy(self.model.q + np.array([0, 0, 0, 0, 0, np.deg2rad(degree)]))

        print(desired_q)
        # Create a jtraj:
        path = rtb.jtraj(self.model.q, desired_q, 50)

        # Send to messange to the robot
        if real_robot:
            self.send_joint_trajectory(path.q, speed)
            
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

    




    
        



        


        