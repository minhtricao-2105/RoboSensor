function SetupEnvironment(baseTr)

% First, let's close all the window:
close all;

% Add Path:
addpath('../Environment');

%% -- Setup the environment:
axis([-3 3.25 -4 2.5 0 3]);
hold on
PlaceObjectModified('workspace.ply', baseTr);

%% -- Setup the vegetable crack:

% Vegetable Rack Stand on the left:
yLeft = [4, 2.5, 1, -0.5];

for y = yLeft
    PlaceObjectModified('vegetable_crack1.ply', baseTr * trotz(pi/2) * transl(-2, y, 0));
end



end