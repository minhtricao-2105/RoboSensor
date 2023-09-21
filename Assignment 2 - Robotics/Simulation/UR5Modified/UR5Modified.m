classdef UR5Modified < OmcronBaseClass
    %% UR5 Universal Robot 5kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)
        plyFileNameStem = 'UR5Modified';
    end


    methods
        %% Constructor
        function self = UR5Modified(baseTr, qHome, workplaceInput)
            % Create the model of the TM5 robot
            self.CreateModel();

            % if there is no special input for base, default value will be applied (=4x4 identity matrix)
            if nargin < 1
                baseTr = eye(4);
            end

            if nargin < 2
                qHome = [0, 0, 0, 0, 0, 0];  % Default the homing position of the robot if not provided
            end

            if nargin < 3
                workplaceInput =  [-1.2, 1.2, -1.2, 1.2, 0, 1.2]; % Default workplace input if not provided
            end

            % Setup the home position of the robot:
            self.homeQ = qHome;

            % Setup the workspace of the robot:
            self.workspace = workplaceInput;

            % The base of the TM5 robot is based on the base input
            self.baseTransform = self.baseTransform * baseTr;

            % Plot the UR3 model on simulation environment
            self.CreateModel();
            self.PlotAndColourRobot();
        end

        %% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.089159,    'a',0,      'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(2) = Link('d',0,         'a',-0.425,  'alpha',0,'offset',0,'qlim',[deg2rad(-180),deg2rad(180)]);
            link(3) = Link('d',0,         'a',-0.39225,'alpha',0,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
            link(4) = Link('d',0.10915,     'a',0,      'alpha',pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(5) = Link('d',0.09465,     'a',0,      'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
            link(6) = Link('d',0.0823,     'a',0,      'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink(link,'name',self.name, 'base', self.baseTransform);
        end
    end
end