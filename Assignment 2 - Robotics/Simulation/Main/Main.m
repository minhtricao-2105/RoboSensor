%% Setup the environment:
clc;
clear all;
close all;

% Add path:
addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');

% Call the setup function:
% SetupEnvironment(eye(4));

% Setup workspace:
% workspace = [-6, 6, -6, 6, 0, 4];
qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];

% Setup the robot:
ttRobotTM5Mobile = TM5Mobile(transl(0,0,0.8)*trotz(pi), qHome);

% ttRobot = TM5(transl(5,0,1), qHome);
