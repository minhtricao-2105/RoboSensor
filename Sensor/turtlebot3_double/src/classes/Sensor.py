# Import
import rospy
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import numpy as np 

# Import the Sensor message
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Sensor:

    # Constructor:
    def __init__(self):
        
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscriber:
        self.image_sub = rospy.Subscriber('/tb3_0/camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/tb3_0/camera/depth/image_raw', Image, self.depth_callback)

        # Publisher:
        self.cmd_vel_pub = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)

        # Data Members of this class:
        self.depth_image = None
        self.rgb_image = None

        # ArUco Setup:
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # Initializing the Twist message:
        self.move_cmd = Twist()

    # Callback function for the subscriber:
    def depth_callback(self, msg):

        # Store the image in the data member:
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    # Callback function for the subscriber:
    def image_callback(self, msg):

        # Store the image in the data member:
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert the ROS message to an OpenCV image:
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detect ArUco markers:
        markerCorners, markerIds, _= cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)

        if markerIds is not None:
            ########################
            #This function is used to get the rotation matrix and translation matrix 
            #for reference: https://docs.opencv.org/4.8.0/d9/d6a/group__aruco.html#ga3bc50d61fe4db7bce8d26d56b5a6428a
            marker_size = 0.1           #replace with real marker size

            fx = 1.085595       #focal length in x axis
            fy = 1.085595       #focal length in y axis
            cx = 320            #principal point x
            cy = 240            #principal point y

            camera_matrix = np.array([[fx, 0, cx],
                                    [0, fy, cy],
                                    [0, 0, 1]], dtype=np.float64)

            
            k1 = 0
            k2 = 0
            p1 = 0
            p2 = 0
            k3 = 0
            
            dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float64)

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(markerCorners, marker_size, camera_matrix, dist_coeffs)
            
            print("rotation ", rvecs)
            print("translation ", tvecs)
            

            #now use rvecs and tvecs for controller
            ########################

            cv2.aruco.drawDetectedMarkers(cv_image, markerCorners, markerIds)

            # Example: Taking the depth value of the first detected marker's center
            center_x = int((markerCorners[0][0][0][0] + markerCorners[0][0][2][0]) / 2)
            center_y = int((markerCorners[0][0][0][1] + markerCorners[0][0][2][1]) / 2)

            # Get the image width and height:
            height, width = cv_image.shape[:2]

            error_x = center_x - width / 2

            # Control stategy:
            # define Kp
            kp = 0.001
            angular_velocity = -kp*error_x
            # Get the depth value from the depth image:
            depth = self.depth_image[center_y][center_x]

            if depth < 0.5:
                linear_velocity = 0.0
            else:
                linear_velocity = 0.2
            
            self.move_cmd.linear.x = linear_velocity
            self.move_cmd.angular.z = angular_velocity
            self.cmd_vel_pub.publish(self.move_cmd)

            # Display the value on the image:
            cv2.putText(cv_image, str(depth), (center_x, center_y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

            # Draw a circle at the center of the marker:
            cv2.circle(cv_image, (center_x, center_y), 5, (0, 0, 255), -1)
        else:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(self.move_cmd)


        # Now you can visualize the image with detected markers using OpenCV
        cv2.imshow('Detected ArUco markers', cv_image)
        cv2.waitKey(1)  # Display the image for a short duration (1 ms). This keeps the display updated.




        