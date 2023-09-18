%% Setup the enronment:
clc;
clear all;
close all;

% Add path:
addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');

qHome = [-173, -97.2, -170, -90, 90, 0]*pi/180;
workspace = [-1.5 1.5, -1.5, 1.5, 0, 2];

% Setup the robot:
ttRobotTM5Mobile = TM5Mobile(transl(0,0,0.8)*trotz(pi), qHome, workspace);

hold on 
% Call the ellipsoid function:

check = ttRobotTM5Mobile.CheckSelfCollision()