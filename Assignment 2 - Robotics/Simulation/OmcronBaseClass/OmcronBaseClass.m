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

        %% GetLinkPoses Function:
        function [transforms] = GetLinkPoses(self, q)

            % Get the links of the robot:
            links = self.model.links;

            % Create a memory variables for the transform:
            transforms = zeros(4, 4, length(links) + 1);

            % Store the transform of the base:
            transforms(:,:,1) = self.model.base;

            % Begin to get the transform of each link from the DH Parameter:
            for i = 1:length(links)

                % Get the DH parameter:
                L = links(1,i);

                % Get the transform of the current link:
                currentTransform = transforms(:,:,i);

                % Calculate the transform of the next link:
                nextTransform = currentTransform*trotz(q(1,i) + L.offset)*transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);

                % Store it to our output of this function:
                transforms(:,:,i+1) = nextTransform;
            end

        end

        %% Check Collision using Line-Plane Intersection
        % Given a robot model (robot), and trajectory (i.e. joint state vector) (qMatrix)
        % and triangle obstacles in the environment (faces,vertex,faceNormals)

        function collisionDetection = CheckColisionLinePlane(self, qMatrix, faces, vertex, faceNormals, returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end

            % Set the initial value for the output variable:
            collisionDetection = false;

            % Begin to check collision:
            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = self.GetLinkPoses(qMatrix);

                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && self.IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
                            display('[WARNING]: Collision Detection!');
                            collisionDetection = true;
                            if returnOnceFound
                                return
                            end
                        end
                    end
                end
            end
        end

        %% IsIntersectionPointInsideTriangle
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is
        % inside (result == 1) or
        % outside a triangle (result ==0 )
        function result = IsIntersectionPointInsideTriangle(self, intersectP,triangleVerts)

            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);

            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);

            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);

            D = uv * uv - uu * vv;

            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end

            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end

            result = 1;                      % intersectP is in Triangle
        end

        %% QuadraticDistance function
        %
        %
        %
        function [distances, conditions] = QuadraticDistance(self, points, centerPoint, radii)

            % Calculate the quadraticDistance:
            distances = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                + ((points(:,3)-centerPoint(3))/radii(3)).^2;

            % Check the conditions for each point:
            conditions = zeros(size(distances));
            for i = 1:length(distances)
                if distances(i) < 1
                    conditions(i) = 0;
                elseif distances(i) > 1
                    conditions(i) = 1;
                else
                    conditions(i) = 2;
                end

            end
        end

        %% CreateRobotEllipsoid
        % This function is used to create a ellipsoid cover the robot
        function [X,Y,Z] = CreateRobotEllipsoid(self, visualize)

            % Check the input argument:
            if nargin < 2
                visualize = false;
            end

            % Get the transform of the robot:
            transforms = self.GetLinkPoses(self.model.getpos);

            % Extract all x, y, z positions of each link end
            xPositions = squeeze(transforms(1, 4, :));
            yPositions = squeeze(transforms(2, 4, :));
            zPositions = squeeze(transforms(3, 4, :));

            % Determine the bounding box of the robot
            minX = min(xPositions);
            maxX = max(xPositions);

            minY = min(yPositions);
            maxY = max(yPositions);

            minZ = min(zPositions);
            maxZ = max(zPositions);

            % Calculate the center and dimensions of the ellipsoid
            centerPoint = [(minX + maxX)/2, (minY + maxY)/2, (minZ + maxZ)/2];
            radii = [((maxX - minX)/2)*1.85, ((maxY - minY + 0.15)/2)*2, ((maxZ - minZ)/2)*1.6];

            % Plot the ellipsoid on the current figure
            [X, Y, Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));

            if visualize == true
                surf(X, Y, Z, 'FaceColor', [0.5 0.5 0.5], 'FaceAlpha', 0.5, 'EdgeColor', 'none');
            end

        end

        %% RMRC:
        % RMRC from the current position to the desired point in the
        % Cartesian plane.
        function RMRC(self, point1, point2)

            % Initial parameter
            t = 2;                              % Total Time of motion
            steps = 50;                         % Number of steps
            deltaT = t/steps;                   % Small Angle Change
            threshold = 0.2;                    % Threshhold value for manipulability
            waypoints = zeros(steps,3);         % Waypoints matrix

            % Linear interpolation between the two points
            for i = 1:steps
                t = (i - 1) / (steps - 1);  % Interpolation parameter [0, 1]
                waypoints(i, :) = (1 - t) * point1 + t * point2;
            end

            % Create a matrix of joint angles
            qMatrix = nan(steps,self.model.n);

            % Solve for Initial Joint Angles
            qMatrix(1, :) = self.model.getpos;
            
            % Get the desired orientation of the end-effector:
            desiredPose = self.model.fkine(self.model.getpos).T;
            desiredOre  = desiredPose(1:3, 1:3);

            % Track the trajectory with RMRC:
            for i = 1:steps-1
                
                % Find the linear velocity vector at this current pose:
                xdot = (waypoints(i + 1, :) - waypoints(i, :)) / deltaT;

                % Find the Jacobian Matrix at the current pose:
                J = self.model.jacob0(qMatrix(i,:));

                % Calculate the different between the current pose and the desired pose:
                orientationDiff = self.CalculateOriDiff(desiredOre, qMatrix(i,:));

                % Combine Velocity:
                combinedVelocity = [xdot';  orientationDiff];
                
                % Measure of Manipulability
                mu = sqrt(det(J*J'));
                
                % If manipulability is less than given threshold
                if mu < threshold 
                    % Damping coefficient (example value)
                    lambda = 0.01; 

                    % Apply Damped Least Squares pseudoinverse
                    invJ = (J'*J + lambda^2*eye(6))\J';
                else
                    invJ = pinv(J);
                end

                % Solve velocities via RMRC:
                qdot = invJ*combinedVelocity;
                
                % Update next joint state based on joint velocities:
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot'; 

                % Animate the path:
                self.model.animate(qMatrix(i+1,:));
                pause(0.05)
            end
        end

        %% orientationDiff
        function orientationDiff = CalculateOriDiff(self, desiredOre, currentPose)

            currentPose = self.model.fkine(currentPose).T;
            currentOre  = currentPose(1:3,1:3);

            % Find the rotation matrix between the desired pose and the current:
            rotationMatrix = desiredOre*currentOre';

            % Convert the rotation matrix to a rotation vector (axis-angle representation)
            angle = rotm2axang(rotationMatrix);

            % Convert the rotation vector to a 3x1 angular error vector
            orientationDiff = angle(:,1:3)'*angle(:,4);
        end

        %% 
        
    end
end