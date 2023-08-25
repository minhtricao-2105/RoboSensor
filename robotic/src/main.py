#!/usr/bin/env python3

from RobotBaseClass import*

rospy.init_node('robot_node')

robot = RobotBaseClass()

rospy.sleep(1)

print(robot.currentQ)

robot.move_jtraj(robot.currentQ, [0, 0, -pi/2, -pi/2, pi/2, 0], speed = 3)

print(robot.model.fkine(robot.currentQ))


