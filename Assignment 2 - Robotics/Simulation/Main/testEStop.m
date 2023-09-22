clc;
clear all;
close all;

addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');
addpath('../Environment')

qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];
workspace = [-1.5 1.5, -1.5, 1.5, 0, 2];

% Setup the robot:
ttRobotTM5Mobile = TM5Mobile(transl(0,0,0.8)*trotz(pi), qHome, workspace);

tr = ttRobotTM5Mobile.model.fkine(ttRobotTM5Mobile.model.getpos).T;
ttRobotTM5Mobile.rmrc(tr, tr*transl(-0.2, -0.3, 0.3), ttRobotTM5Mobile.model.getpos, product1);

tr = ttRobotTM5Mobile.model.fkine(ttRobotTM5Mobile.model.getpos).T;
ttRobotTM5Mobile.rmrc(tr, tr*transl(0.2, 0.3, -0.3), ttRobotTM5Mobile.model.getpos);

