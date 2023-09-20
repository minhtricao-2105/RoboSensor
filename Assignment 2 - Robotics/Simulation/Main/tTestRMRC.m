clear;
close all;
clc;

addpath('../TM5/');
addpath('../OmcronBaseClass/')
addpath('../TM5Mobile/')

testTM5 = TM5(eye(4), [0, -pi/2, -pi/2, -pi/2, pi/2, 0]);

steps = 50;
deltaT = 0.1;

% startPose = transl(-0.5, -0.5, 0.2) * trotx(pi);
startPose = testTM5.model.fkine(testTM5.model.getpos).T;
endPose = transl(0.5, 0.5, 0.3) * trotx(pi);

startPoint = startPose(1:3,4)';
% endPoint = startPoint;
endPoint = endPose(1:3,4)';

path = zeros(3, steps);
s = lspb(0,1,steps);
for i=1:steps
    path(:,i) = startPoint*(1-s(i)) + s(i)*endPoint;
end

qMatrix = nan(steps,6);

% qGuess = [27 -28.8 31 -82.8 -86.4 0];

% newQ = testTM5.model.ikcon(startPose, deg2rad(qGuess));
qMatrix(1,:) = testTM5.model.getpos;

% testTM5.model.animate(newQ);
for i=1:steps-1
    pathdot = (path(:,i+1) - path(:,i))/deltaT;
    % orient = [0; 0; 0];
    % pathPlan = [pathdot; orient];
    J = testTM5.model.jacob0(qMatrix(i,:));
    Jv = J(1:3,:);

    % J = pinv(J);
    qdot = pinv(Jv) * pathdot;
    qMatrix(i+1,:) = qMatrix(i,:) + qdot'*deltaT;

    % testTM5.model.animate(qMatrix(i+1,:));
    % pause(0.1);
end

for i=1:steps
    testTM5.model.animate(qMatrix(i,:));
    pause(0.1);
end
