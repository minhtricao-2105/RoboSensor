# Import the necessary packages:
import numpy as np
import cv2 as cv
import rospy


from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera:

    def __init__(self):
        # D435
        # Define the camera matrix K:
        # self.K = np.array([[619.7423706054688, 0.0, 318.9226379394531],
        #       [0.0, 620.3488159179688, 242.47071838378906],
        #       [0.0, 0.0, 1.0]])

        # D435i
        self.K = np.array([[918.7401733398438, 0.0, 647.2181396484375],
              [0.0, 918.3084716796875, 345.8296203613281],
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
        fx, fy = 918.7401733398438, 918.3084716796875
        cx, cy = 647.2181396484375, 345.8296203613281
            
        # Calculate the 3D point:
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy

        P_camera = np.array([x, y, depth, 1])  # 3D point in the camera frame

        # Transformation between Cam and End-effector:
        T_cam_ee = np.array([[0, 0, 1, 0.0329],
                            [0, -1, 0, -0.0405],
                            [1, 0, 0, 0.0632],
                            [0, 0, 0, 1]])
        
        P_ee = np.dot(T_cam_ee, P_camera)

        return P_ee
    
    # ReSubscribe to the topics:
    def re_subscribe(self):
        self.rgb_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depth_subscriber = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)

    # Detect object
    def detect_object(self, color='blue'):
        if self.latest_rgb is None or self.latest_depth is None:
            print('No RGB or Depth Image image received')
            return None

        # Convert the image to OpenCV format:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
        cv_depth = bridge.imgmsg_to_cv2(self.latest_depth, "passthrough")

        # Convert the image to HSV format:
        hsv = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the colors
        thresholds = {
            'blue': (np.array([100, 50, 50]), np.array([140, 255, 255])),
            'red': (np.array([0, 50, 50]), np.array([10, 255, 255])),
            'white': (np.array([0, 0, 200]), np.array([255, 30, 255])),
            'green': (np.array([35, 50, 50]), np.array([85, 255, 255]))
        }

        lower_threshold, upper_threshold = thresholds.get(color, (None, None))
        if not lower_threshold or not upper_threshold:
            print(f"Color {color} not recognized.")
            return None

        mask = cv.inRange(hsv, lower_threshold, upper_threshold)
        contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

        detected_objects = []

        for contour in contours:
            if cv.contourArea(contour) > 400:  # Arbitrary threshold
                M = cv.moments(contour)
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                depth = 0.285

                # Draw a circle around the detected object
                cv.circle(cv_image, (cx, cy), 10, (0, 0, 255), 2)

                # Optional: Draw bounding rectangle
                x, y, w, h = cv.boundingRect(contour)
                cv.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                print('Detected Object at: ', cx, cy, depth)

                coordinates = (cx, cy, depth)
                detected_objects.append(coordinates)

        # cv.imshow('RGB image', cv_image)
        # cv.waitKey(1)  # Display the image for a short duration (1 ms). This keeps the display updated.

        return detected_objects
      
