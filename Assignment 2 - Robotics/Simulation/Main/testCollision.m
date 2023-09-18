%% Setup the environment:
clc;
clear all;
close all;

% Add path:
addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');
addpath('../Environment');

qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];
workspace = [-1.5 1.5, -1.5, 1.5, 0, 2];

% Setup the robot:
ttRobotTM5Mobile = TM5Mobile(transl(0,0,0.8)*trotz(pi), qHome, workspace);

% Plot a cube:
object = SpawnObject;

% Random Cube
centerpnt = [0.65,0,1.25];
side = 0.5;
plotOptions.plotFaces = true;
[vertex,faces,faceNormals] = object.RectangularPrism(centerpnt-side/2, centerpnt+side/2,plotOptions);

% Create a dummy jtraj:
qDesired1 = [pi, -pi/2, -pi/2, -pi/2, pi/2, 0];
qDesired2 = [-pi, -pi/2, -pi/2, -pi/2, pi/2, 0];

path1 = jtraj(qHome, qDesired1, 50);
path2 = jtraj(qDesired1, qDesired2, 50);

% Combine Path:
combinedTrajectories = [path1; path2];

for i = 1:size(combinedTrajectories,1)
    
    % Check Collision:
    ttRobotTM5Mobile.CheckColisionLinePlane(combinedTrajectories(i,:), faces, vertex, faceNormals, false);
    
    % Animate:
    ttRobotTM5Mobile.model.animate(combinedTrajectories(i,:));
    
    pause(0.05);
end


