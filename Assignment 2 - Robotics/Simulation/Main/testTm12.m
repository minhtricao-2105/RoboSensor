%% Setup the environment:
clc;
clear all;
close all;

% Add path:
addpath('../TM5Mobile');
addpath('../TM5');
addpath('../OmcronBaseClass');
addpath('../UR5Modified/')
addpath('../Camera')
addpath('../TM12')

robot = TM12(eye(4), [0, -pi/2, -pi/2, -pi/2, pi/2, 0])