%% Setup the environment:
clc;
clear all;
close all;

% Add path:
addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');
addpath('../UR5Modified/')
addpath('../TM12/')
addpath('../Camera')
addpath('../Arm/')
addpath('../Human/')
%% 

% Setup the transform of the shelves in the environment:
shelf1BaseTr = transl(-2,-2.55,0.1)*trotz(pi/2);
shelf2BaseTr = transl(-1, -2.55,0.1)*trotz(pi/2);
shelf3BaseTr = transl(0,-2.55,0.1)*trotz(pi/2);

% Call the setup function:
SetupEnvironment(eye(4), shelf1BaseTr, shelf2BaseTr, shelf3BaseTr);

% Setup workspace:
qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];

% Setup the robot:
baseTr  = transl(0,0,0.9)*trotz(pi);
baseTr2 = transl(1.95,0.1,0.85);
baseTr3 = transl(2,1.25,0.90);
baseTr4 = transl(1.5,-3.6,0) * trotz(-pi/2);

ttRobotTM5Mobile = TM5Mobile(baseTr, qHome);
ttRobotTM12 = TM12(baseTr2, qHome);
ttArm = Arm(baseTr3);
ttHuman = Human(baseTr4);

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
product1 = PlaceObjectModified('redProduct.ply', baseTr2*transl(0.5, 0, 0));
product2 = PlaceObjectModified('blueProduct.ply', baseTr2*transl(0.5, 0.25, 0));
product3 = PlaceObjectModified('greenProduct.ply', baseTr2*transl(0.5, -0.25, 0));
%% 

% Make the ttRobot move to the position to pick up the product:
% for i = 1:50
%     ttRobotTM5Mobile.model.base = ttRobotTM5Mobile.model.base.T*transl(0,-0.1,0);
%     pause(0.05);
% end
% 
% startPose = ttRobot.model.fkine(ttRobot.model.getpos()).T;
% endPose = baseTr2*transl(0.5, 0, 0);
% qGuess = ttRobot.model.getpos();
% ttRobot.rmrc(startPose, endPose, qGuess,ttHuman);
% 
% startPose = ttRobot.model.fkine(ttRobot.model.getpos()).T;
% endPose = baseTr2*transl(0.5, 0, 0.3);
% qGuess = ttRobot.model.getpos();
% ttRobot.rmrc(startPose, endPose, qGuess,ttHuman, product1);

% Generate path
tm12EndPose = baseTr2*transl(0.5, 0, 0);
waypoints = GeneratePath(ttRobotTM12, tm12EndPose);

tm5EndPose = baseTr*transl(0.5, 0, 0.1);
waypointsTM5 = GeneratePath(ttRobotTM5Mobile, tm5EndPose);

for i=1:50-1
    startPoseTM12 = waypoints(i, :);
    endPoseTM12 = waypoints(i+1, :);

    startPoseTM5 = waypointsTM5(i,:);
    endPoseTM5 = waypointsTM5(i+1,:);

    ttRobotTM12.rmrc(startPoseTM12, endPoseTM12, ttRobotTM12.model.getpos(),0.8,2, ttHuman, ttArm);
    ttRobotTM5Mobile.rmrc(startPoseTM5, endPoseTM5, ttRobotTM5Mobile.model.getpos(),0.8,2, ttHuman, ttArm);
end

startPose = ttRobotTM12.model.fkine(ttRobotTM12.model.getpos()).T;
startPose = startPose(1:3,4)';

endPose = baseTr2*transl(0.5, 0, 0.3);
endPose = endPose(1:3,4)';

ttRobotTM12.rmrc(startPose, endPose, ttRobotTM12.model.getpos(), 2, 50, ttHuman, ttArm, product1);

%% Fucntion to generate path
function waypointsT = GeneratePath(robot, endPose)
    startPose = robot.model.fkine(robot.model.getpos()).T;

    startPoint = startPose(1:3,4)';
    endPoint = endPose(1:3,4)';
    steps = 50;
    waypoints = zeros(steps,3);
    % Generate path
    for i = 1:steps
        t = (i - 1) / (steps - 1);  % Interpolation parameter [0, 1]
        waypoints(i, :) = (1 - t) * startPoint + t * endPoint;
    end

    waypointsT = waypoints;
end

