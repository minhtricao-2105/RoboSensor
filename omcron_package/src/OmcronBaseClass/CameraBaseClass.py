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
        self.depth_subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

        # Store latest RGB and Depth images
        self.latest_rgb = None
        self.latest_depth = None
        
    # RGB Image Callback:
    def rgb_callback(self, msg):
        try:
            self.latest_rgb = msg
            self.detect_object()
        except Exception as e:
            print(e)
            
    # Depth Image Callback:
    def depth_callback(self, msg):
        try:
            self.latest_depth = msg
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
    
    # Detect object
    def detect_object(self):
        if self.latest_rgb is None:
            print('No RGB or Depth Image image received')
            return None
        
        # Convert the image to OpenCV format:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
        depth_image = bridge.imgmsg_to_cv2(self.latest_depth, desired_encoding="32FC1")

        # Convert the image to HSV format:
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the blue color
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv.inRange(hsv, lower_blue, upper_blue)

        # Find contours:
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        depth_at_centroid = None

        for contour in contours:
            if cv.contourArea(contour) > 400:  # Arbitrary threshold
                M = cv.moments(contour)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])

                # Max number of attempts to re-read depth
                max_attempts = 10
                attempts = 0

                # while depth_at_centroid == 0 and attempts < max_attempts:
                #     depth_at_centroid = depth_image[cy, cx]
                #     attempts += 1

                # # If depth is still 0 after all attempts, handle it appropriately (e.g., skip this iteration, log a warning, etc.)
                # if depth_at_centroid == 0:
                #     print("Warning: Unable to get valid depth after multiple attempts.")
                #     continue  # Skip to the next contour or handle as desired

                # # Convert the depth value to meters
                # depth_in_meters = depth_at_centroid / 1000.0

                # # Convert 2D image point to 3D camera point
                # point_3D = self.project_2D_to_3D(cx, cy, depth_at_centroid)

                # # Draw the depth value on the image
                # font = cv.FONT_HERSHEY_SIMPLEX
                # cv.putText(cv_image, f"{depth_in_meters:.2f}m", (cx, cy-20), font, 0.5, (255, 255, 255), 2)

                # Draw a circle around the detected object
                cv.circle(cv_image, (cx, cy), 10, (0, 0, 255), 2)

                # Optional: Draw bounding rectangle
                x, y, w, h = cv.boundingRect(contour)
                cv.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # Display the image with the drawn circle
                cv.imshow('Detected Object', cv_image)
                cv.waitKey(1)

            # return point_3D


        
