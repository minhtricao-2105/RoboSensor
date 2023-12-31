%% Setup the enronment:
clc;
clear all;
close all;
axis off

% qHome = [0, -90, -127, -169, 93.6, 0]*pi/180;
% qHome = [0, -pi/2, -2.705, -1.884, pi/2, 0];
qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];%-pi/2
% qHome = [0, 0, 0, 0, 0, 0];
workspace = [-1 1, -1, 1, 0, 1];

% Setup the robot:
ttRobotTM5Mobile = TM5(eye(4), qHome, workspace);
% robotTM12 = TM12(eye(4), qHome, workspace);

q1 = [91.8 -79.2 -109 -57.6 10.8 0]*pi/180;
q_Matrix = jtraj(qHome, q1, 50);

% pause(5.0);
hold on 
% for i=1:50
%     ttRobotTM5Mobile.model.animate(q_Matrix(i,:));
%     pause(0.1);
% end
% 
% q2 = [179 -107 -76 -97.2 90 0]*pi/180;
% q_Matrix = jtraj(q1, q2, 50);
% for i=1:50
%     ttRobotTM5Mobile.model.animate(q_Matrix(i,:));
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
%% joystick
try
    joy = vrjoystick(1);
catch ME
    warning('Failed to create joystick object. Exiting function.');
    return;
end
i = 0
while i < 1 
    try
        [axes, buttons, povs] = read(joy);
    catch ME  % Handle the error
        warning('Joystick not connected. Exiting function.');
        return;
    end

    % --- Set up the gain value for controlling velocity:
    P_linear = 0.05;
    P_angular = 0.05;

    % --- Set up linear velocity:
    vx = P_linear*axes(1);
    vy = P_linear*axes(2);
    vz = 0.1*(buttons(4)-buttons(1));
    % if (buttons(4) == 1)
    %     vz = P_linear*buttons(4);
    % elseif (buttons(2) == 1)
    %     vz = -P_linear*buttons(2);
    % end

    % --- Set up angular velocity:
    wx = P_angular*axes(3);
    wy = P_angular*axes(4);
    wz = 0.1*(buttons(3)-buttons(2));
    % if (buttons(3) == 1)
    %     wz = P_angular*buttons(3);
    % elseif (buttons(2) == 1)
    %     wz = -P_angular*buttons(2);
    % end

    % --- Combine the velocity vector from two velocity:
    xDot = [vx;vy;vz;wx;wy;wz];

    % 2 - use J inverse to calculate joint velocity
    currentQ = ttRobotTM5Mobile.model.getpos();
    
    J = ttRobotTM5Mobile.model.jacob0(currentQ);

    threshold = 0.045;

    mu = sqrt(det(J*J'));

    lambda_max = 0.4;

    lambda = 0;

    if mu < threshold
        lambda = (1- (mu/threshold)^2)*lambda_max;
        if mu < 0.032
            disp('[Warning]: The Robot is very near Singularity! Go Back!');
        end
    end

    invJ = J'*inv(J*J'+lambda*eye(6));



    qdot = invJ*xDot;
    % 3 - apply joint velocity to step robot joint angles
    q = currentQ + qdot'*0.2;

    ttRobotTM5Mobile.model.animate(q);
    drawnow
end
%% Ellipsoid
clc;
clear all;
close all;
axis off

% qHome = [0, -90, -127, -169, 93.6, 0]*pi/180;
% qHome = [0, -pi/2, -2.705, -1.884, pi/2, 0];
% qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];%-pi/2
qHome = [0, 0, 0, 0, 0, 0];
workspace = [-1 1, -1, 1, 0, 1];

% Setup the robot:
robotTM5 = TM5(eye(4), qHome, workspace);

% Get radius
robotTM5.GetRadiusEllipsoid();

% Get the number of links
numLinks = robotTM5.model.n;

% Get Transform of each link
transforms = robotTM5.GetLinkPoses(robotTM5.model.getpos);

% For each link, calculate ellipsoid:
for i = 1:numLinks

    % The ellipsoid's center is midway between the link's start and end.
    startPoint = transforms(1:3, 4, i)';
    endPoint = transforms(1:3, 4, i+1)';
    center = -(startPoint - endPoint) / 2;

    % Correct the link 3:
    if i == 2
        Zdirection = orientation(1:3,3);
        center = center + 0.15 * Zdirection';
    elseif (i == 6)
        Zdirection = orientation(1:3,3);
        center = center + 0.08 * Zdirection';
    elseif (i == 1)
        center(3) = center(3) + 0.08;
    end

    orientation = transforms(1:3, 1:3, i+1);
    % Center of ellipsoid for each links
    robotTM5.linkEllipsoid.center{i} = center;
end

% New values for the ellipsoid (guessed these, need proper model to work out correctly)

for i = 1:numLinks
    centerPoint = robotTM5.linkEllipsoid.center{i};
    radii = robotTM5.linkEllipsoid.radii{i};
    [X,Y,Z] = ellipsoid( centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3) );
    robotTM5.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    robotTM5.model.faces{i} = delaunay(robotTM5.model.points{i});    
    warning on;
end

% robot.plot3d([0,0,0]);
robotTM5.model.plot3d([0,0,0,0,0,0]);
axis equal
camlight
robotTM5.model.teach()
