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
        
        colours_to_detect = ['blue', 'red', 'yellow']
        detected_objects = []
        blue_object = []
        red_object = []
        yellow_object = []

        for color in colours_to_detect:
            if (color == 'blue'):
                while not blue_object:
                    blue_object = camera.detect_object(color)
                    detected_objects.append(blue_object)
            elif (color == 'red'):
                while not red_object:
                    red_object = camera.detect_object(color)
                    detected_objects.append(red_object)
            elif (color == 'yellow'):
                while not yellow_object:
                    yellow_object = camera.detect_object(color)
                    detected_objects.append(yellow_object)
                    
        
        for obj in detected_objects:
            for o in obj:
                x, y, depth, label = o   
                point = camera.project_2D_to_3D(x, y, depth)
                print(f"Color Label: {label}, Point: {point}")

        
