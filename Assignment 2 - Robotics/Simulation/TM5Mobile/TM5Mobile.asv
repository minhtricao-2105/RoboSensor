classdef TM5Mobile < OmcronBaseClass
    %% TODO: Write description here:
    %----------------------------------- Properties Hidden -------------------------------%
    properties(Access = public)
        plyFileNameStem = 'TM5';

        % Create a gripper object
        Gripper;

        % Q home pick
        qHomePick;
        
        % Q home place
        qHomePlace;
    end

    methods
        %% Constructor
        function self = TM5Mobile(baseTr, qHome, workplaceInput)
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

            % % Create a griper
            % self.Gripper = gripper(self.baseTransform);
            
            self.qHomePlace = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];

            self.qHomePick = [pi, -pi/2, -pi/2, -pi/2, pi/2, 0];
        end

        %% Create Model Function:
        function CreateModel(self)
            link(1) = Link('d',0.146,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.329,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.3115,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.124,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.110,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.1132,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            self.model = SerialLink(link,'name',self.name, 'base', self.baseTransform);
        end

        %% Move base function
        function MoveBase(self, x, y, yaw, human, app, products)
            % flag stop
            flagStop = false;

            % Check whenever we want to move the object with the robot:
            moveProduct = true;
            
            productsEE = {};
            % If there is no object input => No need to move the object
            if nargin < 7
                moveProduct = false;
            else
                % Get the transformation between the EE and the object
                eeTr = self.model.base.T;
                for i=1:6
                    objectEE = inv(eeTr)*products{i}.baseTr;
                    productsEE{i} = objectEE;
                end
            end
            
            i=1;
            while i < 50
                if (flagStop == false)
                    self.model.base = self.model.base.T*transl(-x/50, -y/50, 0)*trotz(deg2rad(yaw)/50);
                    self.model.animate(self.model.getpos);
                end

                eeTr = self.model.base.T;
                if moveProduct
                    for k=1:6
                        transform = eeTr*productsEE{k};
                        products{k}.moveObject(transform);
                    end
                end
                
                checkCollision = self.HumanCollisionCheck(human);

                if (strcmp(self.robotState, 'stop') || strcmp(self.robotState, 'holding') || checkCollision == true)
                    i = i;
                    disp("EMERGENCY STOP!");

                    % Turn on the Switch Button
                    app.ActivateSafetyMode;
                    
                    % Toggle the Lamp for Warning in the app:
                    if checkCollision == true
                        app.ToggleLampLight();
                    end
                    
                    flagStop = true;
                else
                    i = i + 1;
                    flagStop = false;
                end
                
                % Check if the cancel flag is raised:
 
                if app.cancelFlag == true
                    break;
                end
                pause(0.05);
            end
            
        end

    end
end
