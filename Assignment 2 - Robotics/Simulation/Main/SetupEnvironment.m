function SetupEnvironment(baseTr)

% First, let's close all the window:
close all;

% Add Path:
addpath('../Environment');

%% -- Setup the environment:
axis([-6.3 6.3 -6.3 6.3 0 4]);
hold on
PlaceObjectModified('workspace.ply', baseTr);

%% -- Setup the vegetable crack:

% Vegetable Rack Stand on the left:
yLeft = [5, 3.5, 2, 0.5];

for y = yLeft
    PlaceObjectModified('vegetable_crack1.ply', baseTr * trotz(pi/2) * transl(-5.25, y, 0));
end

% Vegetable Rack Stand on the right:
yRight = [-5, -3.5, -2, -0.5];

for y = yRight
    PlaceObjectModified('vegetable_crack1.ply', baseTr * trotz(-pi/2) * transl(-5.25, y, 0));
end


end