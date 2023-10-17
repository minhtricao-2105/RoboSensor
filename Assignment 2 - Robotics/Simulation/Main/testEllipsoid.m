%% Setup the enronment:
clc;
clear all;
close all;

% qHome = [0, -90, -127, -169, 93.6, 0]*pi/180;
% qHome = [0, -pi/2, -2.705, -1.884, pi/2, 0];
qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];%-pi/2
% qHome = [0, 0, 0, 0, 0, 0];
workspace = [-1 1, -1, 1, 0, 1];

% Setup the robot:
ttRobotTM5Mobile = TM5(eye(4), qHome, workspace);
% robotTM12 = TM12(eye(4), qHome, workspace);

q1 = [-81 -64.8 -121 -90 180 0]*pi/180;
q_Matrix = jtraj(qHome, q1, 50);

ttRobotTM5Mobile.model.plot(qHome)
pause(10.0);
hold on 
for i=1:50
    ttRobotTM5Mobile.model.animate(q_Matrix(i,:));
    pause(0.1);
end
% 
% q2 = [179 -107 -76 -97.2 90 0]*pi/180;
% q_Matrix = jtraj(q1, q2, 50);
% for i=1:50
%     robotTM12.model.animate(q_Matrix(i,:));
%     pause(0.1);
% end
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

% ttRobotTM5Mobile.CheckSelfCollision
% ttRobotTM5Mobile.CreateEllipsoidLinks(true)
% robotTM12.CreateEllipsoidLinks(true)