function SetupEnvironment(baseTr)

% First, let's close all the window:
close all;

% Add Path:
addpath('../Environment');

%% -- Setup the environment:
axis([-5 5 -5 5 0 4]);
hold on
PlaceObjectModified('workspace.ply', baseTr);

%% -- Setup the vegetable crack:

% Vegetable Rack Stand on the left:
yLeft = [4, 2.5, 1, -0.5];

for y = yLeft
    PlaceObjectModified('vegetable_crack1.ply', baseTr * trotz(pi/2) * transl(-4, y, 0));
end

% Vegetable Rack Stand on the right:
yRight = [-4, -2.5, -1, 0.5];

for y = yRight
    PlaceObjectModified('vegetable_crack1.ply', baseTr * trotz(-pi/2) * transl(-4, y, 0));
end

end