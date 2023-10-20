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

        # Wait for the first image:
        first_img_msg = rospy.wait_for_message("/camera/color/image_raw", Image, timeout=10)
        self.rgb_callback(first_img_msg) 

        # Image:
        self.cv_image = None

        self.point = None
        
    # RGB Image Callback:
    def rgb_callback(self, msg):
        self.latest_rgb = msg
            
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

        # translation of camera 
        tx = -0.0329
        ty = 0.0555
        tz = -0.0632
            
        # Calculate the 3D point (with end of effector frame)
        x = (u - cx) * depth / fx
        y = (v - cy) * depth / fy
        
        x = x + tx
        y = y + ty
        depth = depth - tz

        P_camera = np.array([x, y, depth, 1])  # 3D point in the camera frame


        return P_camera
    
    # ReSubscribe to the topics:
    def re_subscribe(self):
        self.rgb_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)

    # Detect object
    def detect_object(self, color):

        if self.latest_rgb is None:
            print('No RGB data')
            return None
        
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")


        color_labels = {
            'blue': 1,
            'red': 2,
            'yellow': 3,
        }

        # Convert the image to HSV format:
        hsv = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the colors
        # -- Define the lower and upper bounds of the blue color
        if color == 'blue':
            lower_threshold = np.array([100, 50, 50])
            upper_threshold = np.array([140, 255, 255])
        elif color == 'red':
            lower_threshold  = np.array([160, 100, 20])
            upper_threshold = np.array([179, 255, 255])
        elif color == 'yellow':
            lower_threshold  = np.array([20, 100, 100])
            upper_threshold = np.array([30, 255, 255])
        elif color == 'green':
            lower_threshold  = np.array([35, 50, 50])
            upper_threshold = np.array([85, 255, 255])
    
        mask = cv.inRange(hsv, lower_threshold, upper_threshold)
        
        # contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        
        # Find the contours of the object:
        contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
        
        # Define the detected objects list:
        detected_objects = []

        for i, c in enumerate(contours):
            # Calculate the area of each contour:
            area = cv.contourArea(c)

            # Ignore contours that are too small or too large:
            if area < 6789 or 100000 < area:
                continue

            rect = cv.minAreaRect(c)
            box = cv.boxPoints(rect)
            box = np.int0(box)

            # Get the center, width, height, and angle of the bounding rectangle:
            cx, cy = int(rect[0][0]), int(rect[0][1])
            depth = 0.285

            rect = cv.minAreaRect(c)
            box = cv.boxPoints(rect)
            box = np.int0(box)

            # Draw the rotated rectangle
            cv.drawContours(self.cv_image, [box], 0, (0, 255, 0), 2)

            # Extract the angle of the rotated rectangle
            angle = rect[-1]

            if angle > 45:
                angle -= 90

            coordinates = (cx, cy, depth, angle, color_labels[color])

            detected_objects.append(coordinates)
        return detected_objects
    
    def detect_object_modified(self):
        
        if self.latest_rgb is None:
            print('No image received')
            return None
        
        bridge = CvBridge()
        self.cv_image = bridge.imgmsg_to_cv2(self.latest_rgb, "bgr8")
        hsv = cv.cvtColor(self.cv_image, cv.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the colors
        color_labels = {
            'blue': 1,
            'red': 2,
            'yellow': 3
        }
        
        color_thresholds = {
            'blue': (np.array([100, 50, 50]), np.array([140, 255, 255])),
            'red': (np.array([160, 100, 20]), np.array([179, 255, 255])),
            'yellow': (np.array([20, 100, 100]), np.array([30, 255, 255]))
        }

        all_detected_objects = {}

        for color, (lower_threshold, upper_threshold) in color_thresholds.items():
            mask = cv.inRange(hsv, lower_threshold, upper_threshold)
            contours, _ = cv.findContours(mask, cv.RETR_LIST, cv.CHAIN_APPROX_SIMPLE)
            detected_objects = []
            
            for i, c in enumerate(contours):
                area = cv.contourArea(c)
                if area < 6789 or 100000 < area:
                    continue

                rect = cv.minAreaRect(c)
                box = cv.boxPoints(rect)
                box = np.int0(box)
                cx, cy = int(rect[0][0]), int(rect[0][1])
                depth = 0.285
                angle = rect[-1]
                if angle > 45:
                    angle -= 90
                coordinates = (cx, cy, depth, angle, color_labels[color])
                detected_objects.append(coordinates)
                cv.drawContours(self.cv_image, [box], 0, (0, 255, 0), 2)

            all_detected_objects[color] = detected_objects

        return all_detected_objects



      