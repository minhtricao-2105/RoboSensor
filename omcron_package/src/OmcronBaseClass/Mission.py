# Import Library Needed:
import numpy as np
import rospy
import roboticstoolbox as rtb 
import copy
import spatialgeometry.geom as collisionObj

class Mission:
    def __init__(self):
        print("Update something")

    def detect_multi_object(self, camera):
        
        colours_to_detect = ['white', 'red', 'yellow']
        detected_objects = None

        for color in colours_to_detect:
            objects_of_color = camera.detect_object(color)
            if objects_of_color:
                detected_objects.extend(objects_of_color)
        
        for obj in detected_objects:
            x, y, depth, label = obj    
            point = camera.project_2D_to_3D(x, y, depth)
            print(f"Color Label: {label}, Point: {point}")

        
