%% Setup the environment:
clc;
clear all;
close all;

% Add path:
addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');

% Call the setup function:
SetupEnvironment(eye(4));

% Setup workspace:
qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];

% Setup the robot:
baseTr  = transl(0,0,0.8)*trotz(pi);
baseTr2 = transl(4,0.25,1);

ttRobotTM5Mobile = TM5Mobile(baseTr, qHome);
ttRobot = UR5(transl(4,0.25, 0.9));
ttRobot.model.animate(qHome);

hold on

% Set up the product:
% Generate random x and y offsets within [-0.5, 0.5] range
randX1 = 0.25 + (0.5-0.25)*rand();
randY1 = -0.5 + (0.5+0.5)*rand();

randX2 = -0.5 + (0.5+0.5)*rand();
randY2 = -0.5 + (0.5+0.5)*rand();

randX3 = -0.5 + (0.5+0.5)*rand();
randY3 = -0.5 + (0.5+0.5)*rand();

% Place products using the random offsets
product1 = PlaceObjectModified('redProduct.ply', baseTr2*transl(0.5, randY1, -0.1));
product2 = PlaceObjectModified('blueProduct.ply', baseTr2*transl(0.5, randY2, -0.1));
product3 = PlaceObjectModified('greenProduct.ply', baseTr2*transl(0.5, randY3, -0.1));

% Make the ttRobot move to the position to pick up the product:
for i = 1:50
    ttRobotTM5Mobile.model.base = ttRobotTM5Mobile.model.base.T*transl(0,-0.1,0);
    pause(0.05);
end
