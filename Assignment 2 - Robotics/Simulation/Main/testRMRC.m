%% Setup the environment:
clc;
clear all;
close all;

addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');

qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];
workspace = [-1.5 1.5, -1.5, 1.5, 0, 2];

% Setup the robot:
ttRobotTM5Mobile = TM5Mobile(transl(0,0,0.8)*trotz(pi), qHome, workspace);

tr = ttRobotTM5Mobile.model.fkine(qHome).T;
point1 =  tr((1:3),4)';
point2 = point1;
point2(3) = point2(3) - 0.3;
% point2(2) = point2(2) + 0.12;

ttRobotTM5Mobile.rmrc(tr, tr*transl(-0.2, -0.3, 0.3), ttRobotTM5Mobile.model.getpos);

