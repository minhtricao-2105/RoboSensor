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
            % Check the input of the user
            if nargin < 1
                baseTr = eye(4); % Default the transformation of the base if not provided
            end

            if nargin < 2
                qHome = [0, 0, 0, 0, 0, 0, 0];  % Default the homing position of the robot if not provided
            end

            if nargin < 3
                workplaceInput =  [-1.2, 1.2, -1.2, 1.2, 0, 1.2]; % Default workplace input if not provided
            end

            % Setup the home position of the robot:
            self.homeQ = qHome;

            % Setup the workspace of the robot:
            self.workspace = workplaceInput;

            % Create Model and Plot and Color the robot:
            self.CreateModel();
            
            self.model.base = self.model.base.T * baseTr;
            
            self.PlotAndColourRobot();

            drawnow
        end

        %% Create a robot model:
        function CreateModel(self)

            % DH Parameter of the TM5:
            link(1) = Link('d',0.142784,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]), 'offset',0);
            link(2) = Link('d',0,'a',-0.327715,'alpha',0,'qlim', deg2rad([-180 180]), 'offset',0);
            % link(3) = Link('d',0,'a',-0.21325,'alpha',0,'qlim', deg2rad([-180 180]), 'offset', 0);
            % link(4) = Link('d',0.11235,'a',0,'alpha',pi/2,'qlim',deg2rad([-180 180]),'offset', 0);
            % link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-180,180]), 'offset',0);
            % link(6) = Link('d',0.0819,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);

            % Create a model for the TM5:
            self.model = SerialLink(link,'name',self.name);
        end

        %% Plot and Colour the Robot:
        function PlotAndColourRobot(self)

            % Check whenever we define the home position or not:
            if isempty(self.homeQ)
                self.homeQ = zeros(1, self.model.n);
            end

            % Read the ply file:
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['TM5Link',num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Plot the robot:
            self.figure_h = gcf;
            self.axis_h = gca;
            initialSurfaceCount = self.CountTiledFloorSurfaces();

            self.model.plot3d(self.homeQ,'noarrow','nowrist','workspace',self.workspace);

            % Check if a single surface has been added by plot3d
            if self.CountTiledFloorSurfaces() - initialSurfaceCount == 1
                self.surfaceAdded = true;
            end

            % Check if a light needs to be added
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
                self.lightAdded = true;
            end

            self.model.delay = 0;
            handles = findobj('Tag', self.model.name);
            h = get(handles,'UserData');

            % Colour the robot here:
            for linkIndex = 0:self.model.n
                vertexColours = [0.2, 0.5, 0.8]; % Default if no colours in plyData
                try
                    vertexColours = [plyData{linkIndex+1}.vertex.red ...
                        , plyData{linkIndex+1}.vertex.green ...
                        , plyData{linkIndex+1}.vertex.blue]/255;

                catch ME_1
                    disp(ME_1);
                    disp('No vertex colours in plyData');
                    try
                        vertexColours = [plyData{linkIndex+1}.face.red ...
                            , plyData{linkIndex+1}.face.green ...
                            , plyData{linkIndex+1}.face.blue]/255;
                    catch ME_1
                        disp(ME_1);
                        disp(['Also, no face colours in plyData, so using a default colour: ',num2str(vertexColours)]);
                    end
                end

                h.link(linkIndex+1).Children.FaceVertexCData = vertexColours;
                h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
            drawnow();
        end

        %% CountTiledFloorSurfaces
        % A way of checking if a base tiled floor surface has been added
        % that needs deleting
        function surfaceCount = CountTiledFloorSurfaces(self)
            surfaceCount = numel(findobj(self.axis_h, 'Type', 'surface', 'Tag', 'tiled_floor'));
        end
    end
end