#!/usr/bin/env python

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from apriltag_ros.msg import AprilTagDetectionArray  # Import AprilTag messages

class RealSense:

    # Constructor:
    def __init__(self):
        # Create a CvBridge object
        self.bridge = CvBridge()

        # Subscriber:
        self.imageSub = rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_callback)
        self.depthSub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.apriltagSub = rospy.Subscriber("/tag_detections", AprilTagDetectionArray, self.apriltag_callback)

        # Data Members of this class:
        self.depth_image = None
        self.rgb_image = None
        self.apriltag_detections = None  # Store AprilTag detections

        self.depth = None

    # Callback function for the subscriber:
    def depth_callback(self, msg):

        # Store the image in the data member:
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Access depth values at specific pixel coordinates (x, y)
        # x = 424  # Replace with the desired x-coordinate (pixel column)
        # y = 240  # Replace with the desired y-coordinate (pixel row)

        # self.depth = self.depth_image[y][x]  # Access the depth value at (x, y)

        # Display depth image (optional)
        # cv2.imshow("Depth Image", self.depth_image)
        # cv2.waitKey(1)

    def rgb_callback(self, msg):

        # Store the image in the data member:
        # self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Convert the image to HSV format:         
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the lower and upper bounds of the blue color
        lower_blue = np.array([100, 50, 50])
        upper_blue = np.array([140, 255, 255])

        lower_yellow = np.array([20, 100, 100])  # Adjust these values as needed
        upper_yellow = np.array([40, 255, 255])  # Adjust these values as needed
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Find contours:
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:             

            if cv2.contourArea(contour) > 400:

                M = cv2.moments(contour)                

                cx = int(M['m10'] / M['m00'])                 

                cy = int(M['m01'] / M['m00'])

                cv2.circle(cv_image, (cx, cy), 10, (0, 0, 255), 2)

                x, y, w, h = cv2.boundingRect(contour)

                cv2.rectangle(cv_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

                # cv2.drawContours(cv_image, contours, -1, (0, 255, 0), 2)  # (0, 255, 0) is the color, 2 is the thickness

                self.depth = self.depth_image[cy][cx]

                cv2.putText(cv_image, str(self.depth), (cx , cy ), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                cv2.imshow('Detected Object', cv_image)

                cv2.waitKey(1)

    def apriltag_callback(self, msg):
        # Store the AprilTag detections
        self.apriltag_detections = msg.detections