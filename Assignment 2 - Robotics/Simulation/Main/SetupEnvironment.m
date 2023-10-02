function SetupEnvironment(baseTr, baseTr1, baseTr2, baseTr3)

% First, let's close all the window:
close all;

% Add Path:
addpath('../Environment');

%% -- Setup the environment:
axis([-3 3.25 -4 2.5 -0.1 3]);
hold on
PlaceObjectModified('workspace.ply', baseTr);

%% -- Setup the background of the base:
hold on;
surf([-4, -4; 4, 4] ...
,[-4, 4; -4, 4] ...
,[-0.1,-0.1;-0.1,-0.1] ...
,'CData',imread('concrete.jpg') ...
,'FaceColor','texturemap');

%% -- Setup the vegetable crack:
PlaceObjectModified('vegetable_crack1.ply', baseTr1);
PlaceObjectModified('vegetable_crack2.ply', baseTr2);
PlaceObjectModified('vegetable_crack3.ply', baseTr3);

%% -- Setup the light sensor:
for i=1:10
    PlaceObjectModified('laserLight.ply', baseTr * transl(1, -2.9, i*0.2));
end

%% -- Setup the Fire Extructer:
PlaceObjectModified('fireExtinguisher.ply', baseTr * transl(0.8, -3.3, 0));
PlaceObjectModified('fireExtinguisher.ply', baseTr * transl(2.2, -3.3, 0));
end