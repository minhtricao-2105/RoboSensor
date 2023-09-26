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
        linkEllipsoid = struct('center', {{}}, 'radii', {{}}, 'A', {{}}, 'X', {{}}, 'Y', {{}}, 'Z', {{}});
        
        % Setup the State of the robot:
        robotState = 'normal'; %'normal', 'stop', 'holding'

        % Check obstacle avoidance
        obstacleAvoidance = false;
        
        % Check AvoidCollision
        avoidArmCheck = true;
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

            self.model.plot3d(self.homeQ,'noarrow','nowrist','notiles','workspace',self.workspace);

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

        % -------------------- Check Colision in this Area ---------------%

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
                            disp('[WARNING]: Collision Detection!');
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

        %% CreateEllipsoidLinks
        function CreateEllipsoidLinks(self, visualize)

            % Check the input argument of this function:
            if nargin < 2
                visualize = false;
            end

            self.GetRadiusEllipsoid();

            % Get the number of links
            numLinks = self.model.n;

            % Get Transform of each link
            transforms = self.GetLinkPoses(self.model.getpos);

            % For each link, calculate ellipsoid:
            for i = 1:numLinks

                % The ellipsoid's center is midway between the link's start and end.
                startPoint = transforms(1:3, 4, i)';
                endPoint = transforms(1:3, 4, i+1)';
                center = (startPoint + endPoint) / 2;

                % Correct the link 3:
                if i == 2
                    Zdirection = orientation(1:3,3);
                    center = center + 0.15 * Zdirection';
                end

                % Center of ellipsoid for each links
                self.linkEllipsoid.center{i} = center;

                % Define radius for each ellipsoids
                semiAxes = self.linkEllipsoid.radii{i};

                % Generate the ellipsoids for each links
                [X,Y,Z] = ellipsoid(center(1), center(2), center(3), semiAxes(1), semiAxes(2), semiAxes(3));

                % Get the orientation:
                orientation = transforms(1:3, 1:3, i+1);

                for j = 1:numel(X)

                    % Shift to origin
                    point = [X(j); Y(j); Z(j)] - center';

                    % Rotate and shift back
                    rotatedPoint = orientation*point + center';
                    X(j) = rotatedPoint(1);
                    Y(j) = rotatedPoint(2);
                    Z(j) = rotatedPoint(3);
                end

                % Store it the output of this function:
                self.linkEllipsoid.X{i} = X;
                self.linkEllipsoid.Y{i} = Y;
                self.linkEllipsoid.Z{i} = Z;

                % Get the matrix that defines the shape of the ellipsoid
                A = diag([semiAxes(1)^-2, semiAxes(2)^-2, semiAxes(3)^-2]);
                self.linkEllipsoid.A{i} = orientation * A * orientation';


                % Update the ellipsoid with the rotated points:
                if visualize == true
                    surf(X,Y,Z, 'FaceColor', [0,0,1], 'EdgeColor', 'none');
                    hold on;
                end

            end

        end

        %% Define radius
        function GetRadiusEllipsoid(self)
            q = zeros(1,self.model.n);
            tr = GetLinkPoses(self, q);

            sizeOfLinks = size(tr);

            for i=1:sizeOfLinks(3)-1
                base = tr(1:3,4,i)';

                frame = tr(1:3,4,i+1)';

                radius = (frame - base)/2;

                centerPoint = (frame + base)/2;

                if(centerPoint(1) == base(1) && centerPoint(3) == base(3))
                    radii = [0.08, radius(2), 0.08];

                elseif(centerPoint(3) == base(3))
                    radii = [radius(1), 0.08, 0.08];

                elseif(centerPoint(1) == base(1))
                    radii = [0.08, 0.08, radius(3)];
                end
                self.linkEllipsoid.radii{i} = radii;
            end
        end


        %% CheckSelfCollision
        function isCollision = CheckSelfCollision(self)

            % Get the ellipsoids for all links
            linkEllipsoids = self.CreateEllipsoidLinks(false);

            % Number of ellipsoids (equal to number of links)
            numEllipsoids = length(linkEllipsoids);

            % Loop over all pairs of ellipsoids to check for collision
            for i = 1:numEllipsoids-2
                for j = i+2:numEllipsoids

                    % Get the center and radii of the first ellipsoid
                    center1 = linkEllipsoids(i).center;
                    radii1 = linkEllipsoids(i).radii;

                    % Extract the X, Y, and Z coordinates of the points on the second ellipsoid
                    points2 = [linkEllipsoids(j).X(:), linkEllipsoids(j).Y(:), linkEllipsoids(j).Z(:)];

                    % Check if any point from ellipsoid j is inside ellipsoid i
                    [~, conditions] = self.QuadraticDistance(points2, center1, radii1);

                    % Count the number of points that are inside the ellipsoid
                    collisionPoints = sum(conditions == 0)

                    % If any point has a condition of 0, it indicates a collision
                    if collisionPoints > 20
                        isCollision = true;
                        return;
                    end
                end
            end

            % If no collisions detected, return false
            isCollision = false;

        end

        % --------------- Trajectory and Inverse Kinematic is here -------%
        %% SolveValidIkine
        function qFinal = SolveValidIkine(self, desiredT, qGuess)

            % Generate variations around the current configuration
            deltaRot = pi/8;

            variations = [
                deltaRot, 0, 0, 0, 0, 0;
                -delta_rot, 0, 0, 0, 0, 0;
                0, delta_rot, 0, 0, 0, 0;
                0, -delta_rot, 0, 0, 0, 0;
                0, 0, delta_rot, 0, 0, 0;
                0, 0, -delta_rot, 0, 0, 0;
                ];

            % Add variations to current configuration
            guesses = bsxfun(@plus, qGuess, variations);

            % Generate solutions based on the guesses above:
            solutions = [];
            for i = 1:size(guesses, 1)
                q = self.model.ikcon(desiredT, guesses(i, :));
                if ~isempty(q)
                    solutions = [solutions; q];
                end
            end
            % If no solutions found, return an error
            if isempty(solutions)
                error('No IK solutions found');
            end

            % Separate solutions into elbow up and others
            elbowUpSolutions = solutions(solutions(:,3) > 0, :);
            otherSolutions = solutions(solutions(:,3) <= 0, :);

            % Choose the best "elbow up" solution, or if none exist, the best other solution
            if ~isempty(elbowUpSolutions)
                distances = sum((elbowUpSolutions - currentQ).^2, 2);
                [~, idx] = min(distances);
                qFinal = elbowUpSolutions(idx, :);
            else
                distances = sum((otherSolutions - currentQ).^2, 2);
                [~, idx] = min(distances);
                qFinal = otherSolutions(idx, :);
            end
        end
        
        %% AnimatePath
        function AnimatePath(self, newQ, steps, humanObject, obstacleObject, app, object)

            % Setup the initial parameter:
            moveObject = true;
            
            % If there is no object input => No need to move the object
            if nargin < 6
                moveObject = false;
            else
                % Get the transformation between the EE and the object
                eeTr = self.model.fkine(self.model.getpos).T;
                objectEE = inv(eeTr)*object.baseTr;
            end
            
            % Get current joint angle
            currentQ = self.model.getpos();

            % Create path to move from current pose to desired pose
            qMatrix = jtraj(currentQ, newQ, steps);

            % Animate robot move simutaneously with the product
            
            % Animate the path:
            i=1;
            while i < steps
                self.model.animate(qMatrix(i,:));
                eeTr = self.model.fkine(qMatrix(i,:)).T;

                if moveObject
                    transform = eeTr*objectEE;
                    object.moveObject(transform);
                end

                % Check human intersect with workspace
                checkCollision = self.HumanCollisionCheck(humanObject);

                % Check stop 
                if (strcmp(self.robotState, 'stop') || strcmp(self.robotState, 'holding') || checkCollision == true)
                    i = i;
                    disp("EMERGENCY STOP!");

                    % Turn on the Switch Button
                    app.ActivateSafetyMode;
                    
                    % Toggle the Lamp for Warning in the app:
                    if checkCollision == true
                        app.ToggleLampLight();
                    end

                else
                    i = i + 1;
                end

                % Check obstacle, then avoid collision
                if self.avoidArmCheck == true
                    checkObstacle = self.CheckRobotArmObstacle(obstacleObject);
                    if checkObstacle == true
                        self.obstacleAvoidance = true;
                        break;
                    end
                end

                % Update the data of the robot to the gui:
                app.UpdateJointStateData();
                app.UpdateEndEffectorData();

                pause(0.05)
                
            end
        end

        
        %% Move
         
        % --------------- RMRC CONTROL BELOW THIS AREA -------------------%

        %% RMRC:
        % RMRC from the current position to the desired point in the
        % Cartesian plane.


        function rmrc(self, startPose, endPose, qGuess, totalTime, totalSteps, humanObject, obstacleObject, app, object)
            
            % Check whenever we want to move the object with the robot:
            moveObject = true;
            
            % If there is no object input => No need to move the object
            if nargin < 10
                moveObject = false;
            else
                % Get the transformation between the EE and the object
                eeTr = self.model.fkine(self.model.getpos).T;
                objectEE = inv(eeTr)*object.baseTr;
            end

            % Set up the initial parameters:
            t = totalTime;                              %<! Total Time of motion
            steps = totalSteps;                         %<! Number of steps
            deltaT = t/steps;                           %<! Small Angle Change
            threshold = 0.2;                            %<! Threshhold value for manipulability
            waypoints = zeros(steps,3);                 %<! Waypoints matrix

            % Allocate array data:
            qMatrix = zeros(steps, self.model.n);      % Array for joint Angels:

            % Set up trajectory:
            startPoint = startPose;
            endPoint = endPose;

            for i = 1:steps
                t = (i - 1) / (steps - 1);  % Interpolation parameter [0, 1]
                waypoints(i, :) = (1 - t) * startPoint + t * endPoint;
            end

            % Solve joint angles to achieve first waypoint:
            qMatrix(1,:) = self.model.getpos;

            % Animate the first q
            self.model.animate(qMatrix(1,:));

            % Track the trajectory with RMRC:
            i=1;
            
            while i < steps

                % Get forward transformation at current joint state:
                T = self.model.fkine(qMatrix(i,:)).T;

                % Get Position Error from next waypoint:
                deltaX = waypoints(i+1, :) - T(1:3,4)';

                % Get the linear velocity:
                linearVelocity = (1/deltaT)*deltaX;

                % Get the angular velocity:
                angularVelocity = [0,0,0];

                % Calculate the end-effector velocity to reach next waypoint:
                xDot = [linearVelocity'; angularVelocity'];

                % Find the Jacobian Matrix at the current pose:
                J = self.model.jacob0(qMatrix(i,:));

                % Measure of Manipulability
                mu = sqrt(det(J*J'));

                % If manipulability is less than given threshold
                if mu < threshold
                    % Damping coefficient (example value)
                    lambda = 0.01;

                    % Apply Damped Least Squares pseudoinverse
                    invJ = J'*inv(J*J'+lambda*eye(6));
                else
                    invJ = pinv(J);
                end

                % Solve velocities via RMRC:
                qdot = invJ*xDot;

                % Consider the joint limit:
                for j = 1:self.model.n
                    if qMatrix(i,j) + deltaT*qdot < self.model.qlim(j,1)
                        qdot = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot > self.model.qlim(j,2)
                        qdot = 0; % Stop the motor
                    end
                end

                % Update next joint state based on joint velocities:
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot';

                % Animate the path:
                self.model.animate(qMatrix(i+1,:));
                
                % Get the endeffector pose:
                eeTr = self.model.fkine(qMatrix(i+1,:)).T;
                
                % Animate the movement of the object:
                if moveObject
                    transform = eeTr*objectEE;
                    object.moveObject(transform);
                end
                
                % Check human intersect with workspace
                checkCollision = self.HumanCollisionCheck(humanObject);
                
                % Check stop 
                if (strcmp(self.robotState, 'stop') || strcmp(self.robotState, 'holding') || checkCollision == true)
                    i = i;
                    disp("EMERGENCY STOP!");

                    % Turn on the Switch Button
                    app.ActivateSafetyMode;
                    
                    % Toggle the Lamp for Warning in the app:
                    if checkCollision == true
                        app.ToggleLampLight();
                    end

                else
                    i = i + 1;
                end

                % Check obstacle, then avoid collision
                if self.avoidArmCheck == true
                    checkObstacle = self.CheckRobotArmObstacle(obstacleObject);
                    if checkObstacle == true
                        self.obstacleAvoidance = true;
                        break;
                    end
                end
           
                % Update the data of the robot to the gui:
                app.UpdateJointStateData();
                app.UpdateEndEffectorData();

                pause(0.05)

            end 
           
        end

        %% Enable E-Stop    (call back function from GUI)
        function EnableEStop(self)
            self.robotState = 'stop';
        end

        %% Diengaging e-stop    (call back function from GUI)
        function DiengagingEStop(self)
            if (strcmp(self.robotState, 'stop'))
                self.robotState = 'holding';
            end
        end

        %% Resuming robot action    (call back function from GUI)
        function ResumingRobot(self)
            if (strcmp(self.robotState, 'holding'))
                self.robotState = 'normal';
            end
        end

        %% Check collision for human and light curtain
        function checkCollision = HumanCollisionCheck(self, humanObject)
            % Get start point of human model
            currentBase = humanObject.model.base.T;
            startPoint = currentBase(1:3,4)';
            
            % Get end point of human model
            currentQ = humanObject.model.getpos();
            currentPose = humanObject.model.fkine(currentQ).T;
            endPoint = currentPose(1:3,4)';
            
            pointOnPlane = [1.5 -2.9 1];
            planeNormal = [0 1 0];

            [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,startPoint,endPoint);

            if check == 1
                checkCollision = true;
                self.robotState = 'stop';
            else
                checkCollision = false;
            end


        end
        
        %% Check collision for robot arm and obstacle
        function checkObstacle = CheckRobotArmObstacle(self, obstacleObject)
            % Get start point of obstacle object
            currentBase = obstacleObject.model.base.T;
            startPoint = currentBase(1:3,4)';

            % Get end point of obstacle model
            currentQ = obstacleObject.model.getpos();
            currentPose =  obstacleObject.model.fkine(currentQ).T;
            endPoint = currentPose(1:3,4)';

            % Get end of effector of the robot
            eeRobot = self.model.fkine(self.model.getpos).T;
            eeRobot = eeRobot(1:3,4)';

            pointOnPlane = [2 0.75 1];
            planeNormal = [0 1 0];

            [intersectionPoint,check] = LinePlaneIntersection(planeNormal,pointOnPlane,startPoint,endPoint);
            
            distance = norm(intersectionPoint-eeRobot);

            if ((check == 1) && (distance < 0.2))
                checkObstacle = true;
            else
                checkObstacle = false;
            end
        end
        
        
    end
end