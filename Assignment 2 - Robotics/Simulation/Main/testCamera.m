%% Setup the environment:
clc;
clear all;
close all;

% Add path:
addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');
addpath('../UR5Modified/')
addpath('../Camera')

% Call the robot:
robot = UR5Modified(eye(4), [0, -pi/2, -pi/2, -pi/2, pi/2, 0]);
cam = CameraBaseClass(robot.model.fkine(robot.model.getpos).T);

cam.camera_h.ishold
