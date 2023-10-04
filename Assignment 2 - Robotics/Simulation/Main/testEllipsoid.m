%% Setup the enronment:
clc;
clear all;
close all;

% qHome = [-173, -97.2, -170, -90, 90, 0]*pi/180];
qHome = [0, -pi/2, -2.705, -1.884, pi/2, 0];
% qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];
workspace = [-1 1, -1, 1, 0, 1];

% Setup the robot:
ttRobotTM5Mobile = TM5(eye(4), qHome, workspace);

hold on 
% Call the ellipsoid function:
% qDesired = qHome;
% qDesired(1) = qDesired(1) + pi/2;
% path = jtraj(qHome, qDesired, 50);
% 
% for i = 1:length(path)
%     ttRobotTM5Mobile.model.animate(path(i,:));
%     ttRobotTM5Mobile.CheckSelfCollision();
%     pause(0.1);
% end

ttRobotTM5Mobile.CheckSelfCollision