classdef OmcronBaseClass < handle

    properties (Abstract) % An inheriing class must implement and assign these properties
        plyFileNameStem;
    end

    properties
        % Setup the workspace of the robot:
        workspace = []

        % Setup the home position (need to calibrate)
        homeQ = [];

        % Setup the name of the robot:
        name;

        % Define the robot model in this class:
        model;

        % Setup the define transformation:
        baseTransform = eye(4);

        % The constraint variable of the robot platform:
    end

    properties (Hidden)
        axis_h = [];
        figure_h = [];
        lightAdded = false;
        surfaceAdded = false;
    end

    methods
        %% Constructor:
        function self = OmcronBaseClass()
            % This is intentionally left empty. Implement the class
            % constructor in the inhereting class.
            pause(0.001);
            try
                self.name = [self.plyFileNameStem,datestr(now,'yyyymmddTHHMMSSFFF')];
            catch
                self.name = ['RobotBaseClass',datestr(now,'yyyymmddTHHMMSSFFF')];
                warning(['Please include a variable called plyFileNameStem in your inherreting class. For now the robot is named: ',self.name])
            end

        end

        %% CountTiledFloorSurfaces
        % A way of checking if a base tiled floor surface has been added
        % that needs deleting
        function surfaceCount = CountTiledFloorSurfaces(self)
            surfaceCount = numel(findobj(self.axis_h, 'Type', 'surface', 'Tag', 'tiled_floor'));
        end

        %% Plot and Colour the Robot:
        function PlotAndColourRobot(self)

            % Check whenever we define the home position or not:
            if isempty(self.homeQ)
                self.homeQ = zeros(1, self.model.n);
            end

            % Read the ply file:
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread([self.plyFileNameStem,'Link', num2str(linkIndex),'.ply'],'tri');
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Plot the robot:
            self.figure_h = gcf;
            self.axis_h = gca;
            initialSurfaceCount = self.CountTiledFloorSurfaces();

            self.model.plot3d(self.homeQ,'noarrow','notiles','workspace',self.workspace);

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
    end
end