%% Setup the environment:
clc;
clear all;
close all;

qHome = [pi, -pi/2, -pi/2, -pi/2, pi/2, 0];
workspace = [-1.5 1.5, -1.5, 1.5, 0, 2];

% Setup the robot:
ttRobotTM5Mobile = TM5Mobile(transl(0,0,0.8)*trotz(pi), qHome, workspace);
hold on 
% Call the ellipsoid function:
ttRobotTM5Mobile.CreateRobotEllipsoid(true);