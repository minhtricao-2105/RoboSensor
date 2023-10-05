# Import the necessary packages:
import numpy as np
import cv2 as cv
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera:

    def __init__(self):

        # Define the camera matrix K:
        self.K = np.array([[619.7423706054688, 0.0, 318.9226379394531],
              [0.0, 620.3488159179688, 242.47071838378906],
              [0.0, 0.0, 1.0]])
        
        # Define the distortion coefficients:
        self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0])

        # Define the subcriber:
        self.rgb_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_subscriber = rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_callback)

        # Store latest RGB and Depth images
        self.latest_rgb = None
        self.latest_depth = None
        
    # RGB Image Callback:
    def rgb_callback(self, msg):
        try:
            self.latest_rgb = msg
        except Exception as e:
            print(e)
            
    # Projecting a 3D point to a 2D image plane:
    def project_3D_to_2D(self, point_3D, transofromation_matrix):
        
        # Get the point in the camera frame:
        point_3D = np.dot(transofromation_matrix, point_3D)

        # Project the 3D point to the 2D image plane:
        pixel_coords = cv.projectPoints(np.array([point_3D[:3]]), np.zeros((3, 1)), np.zeros((3, 1)), self.K, self.D)[0].reshape(2,)

        return pixel_coords
    
    # Projecting a 2D point to a 3D image plane:
    def project_2D_to_3D(self, u, v, depth):

        # Extract intrinstic parameters:
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]
        
        # Calculate the 3D point:
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy

        P_camera = np.array([x, y, depth, 1])  # 3D point in the camera frame

        return P_camera
