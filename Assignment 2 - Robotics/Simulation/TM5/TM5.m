classdef TM5 < OmcronBaseClass
    %% TODO: Write description here:
    %----------------------------------- Properties Hidden -------------------------------%
    
    properties(Access = public)
        plyFileNameStem = 'TM5Stand';
    end

    methods
        %% Constructor
        function self = TM5(baseTr, qHome, workplaceInput)
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

        %% Create Model Function:
        function CreateModel(self)
            link(1) = Link('d',0.146,'a',0,'alpha',pi/2,'qlim',deg2rad([-270, 270]), 'offset',0);
            link(2) = Link('d',0,'a',-0.329,'alpha',0,'qlim', deg2rad([-180, 180]), 'offset',0);
            link(3) = Link('d',0,'a',-0.3115,'alpha',0,'qlim', deg2rad([-155, 155]), 'offset', 0);
            link(4) = Link('d',0.124,'a',0,'alpha',pi/2,'qlim',deg2rad([-180, 180]),'offset', 0);
            link(5) = Link('d',0.110,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
            link(6) = Link('d',0.1132,'a',0,'alpha',0,'qlim',deg2rad([-270, 270]), 'offset', 0);

            self.model = SerialLink(link,'name',self.name, 'base', self.baseTransform);
        end

        %% Add suction cup model
        function SuctionCup(self)
            hold on;
            mesh_h = PlaceObject('suction_cup.ply');
            axis equal
            vertices = get(mesh_h,'Vertices');
            
            tr = self.model.base.T * transl(-0.6,-0.2,0.02);
            transformedVertices = [vertices,ones(size(vertices,1),1)] * tr';
            set(mesh_h,'Vertices',transformedVertices(:,1:3));
            drawnow();
        end


    end
end
