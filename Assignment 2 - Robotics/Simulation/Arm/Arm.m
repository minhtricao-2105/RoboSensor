classdef Arm < OmcronBaseClass
    %% UR5 Universal Robot 5kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)
        plyFileNameStem = 'Arm';
    end


    methods
        %% Constructor
        function self = Arm(baseTr, qHome, workplaceInput)
            % Create the model of the TM5 robot
            self.CreateModel();

            % if there is no special input for base, default value will be applied (=4x4 identity matrix)
            if nargin < 1
                baseTr = eye(4);
            end

            if nargin < 2
                qHome = [0];  % Default the homing position of the robot if not provided
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
            link(1) = Link('d',0,    'a',0.6,      'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

            self.model = SerialLink(link,'name',self.name, 'base', self.baseTransform);
        end
    end
end