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

        
