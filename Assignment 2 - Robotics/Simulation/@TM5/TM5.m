classdef TM5 < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    %----------------------------------- Properties -------------------------------%
    properties

        % Setup the workspace of the robot:
        workspace;

        % Setup the home position (need to calibrate)
        homeQ;

        % Setup the name of the robot:
        name = ['TM5',datestr(now,'yyyymmddTHHMMSSFFF')];

        % Define the robot model in this class:
        model;

        % Setup the define transformation:
        baseTransform = eye(4);

    end

    %----------------------------------- Properties Hidden -------------------------------%
    properties (Hidden)
        axis_h = [];
        figure_h = [];
        lightAdded = false;
        surfaceAdded = false;
    end


    methods

        %% Constructor of this class:
        function self = TM5(baseTr, qHome, workspaceInput)

        end

        %% Create a robot model:
        function CreateModel(self)

            % DH Parameter of the TM5:
            link(1) = Link('d',0.1519,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            link(2) = Link('d',0,'a',-0.24365,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',0);
            link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', 0);
            link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]),'offset', 0);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
            link(6) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            % Create a model for the TM5:
            self.model = SerialLink(link,'name',self.name, 'base', self.baseTransform);
        end

    end
end