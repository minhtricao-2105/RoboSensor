# Import Library Needed:
import numpy as np
import rospy
import roboticstoolbox as rtb 
import copy
import spatialgeometry.geom as collisionObj

class Mission:
    def __init__(self):
        print("Update something")

    def detect_objects_by_color(self, camera, color):
        """
        Detect objects by color

        Args:
            camera (Camera): Camera object
            color (string): Color of the object
        """
        detected_objects = []
        while not detected_objects:
            detected_objects = camera.detect_object(color)

        points = []

        for obj in detected_objects:
            x, y, depth, angle, label = obj
            point_temp = camera.project_2D_to_3D(x, y, depth)
            point_temp[2] -= 0.18
            point_temp[4] = label
            point_temp[3] = angle
            points.append(point_temp)

        return points

    def detect_multi_object(self, camera):
        """
        Detect multiple objects
        
        Args:
            camera (Camera): Camera object
        """
        colors = ['blue', 'yellow', 'red']
        all_points = []

        for color in colors:
            all_points.append(self.detect_objects_by_color(camera, color))

        return all_points

        
    def detect_multi_object_long(self, camera):
        # Detected point container
        point = []

        #--- Detect blue object
        detected_objects_blue = []

        while not detected_objects_blue:
            detected_objects_blue = camera.detect_object('blue')

        for K in detected_objects_blue:
            x, y, depth, angle, label = K
            point_temp = camera.project_2D_to_3D(x, y, depth)
            point_temp[3] = angle
            point_temp[4] = label
            point.append(point_temp)

        rospy.sleep(0.5)

        #--- Detect yellow object 
        detected_objects_yellow = []

        while not detected_objects_yellow:
            detected_objects_yellow = camera.detect_object('yellow')

        for K in detected_objects_yellow:
            x, y, depth, angle, label = K
            point_temp = camera.project_2D_to_3D(x, y, depth)
            point_temp[3] = angle
            point_temp[4] = label
            point.append(point_temp)

        rospy.sleep(0.5)
        
        #--- Detect red object 
        detected_objects_red = []

        while not detected_objects_red:
            detected_objects_red = camera.detect_object('red')

        for K in detected_objects_red:
            x, y, depth, angle, label = K
            point_temp = camera.project_2D_to_3D(x, y, depth)
            point_temp[3] = angle
            point_temp[4] = label
            point.append(point_temp)

        for i in range(len(point)):
            point[i][2] = point[i][2] - 0.18
        
        return point
        