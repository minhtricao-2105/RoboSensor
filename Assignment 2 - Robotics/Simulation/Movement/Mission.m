classdef Mission < handle

    properties
        mainAppHandle = [];
    end

    methods
        
        %% Constructor of this class:
        function self = Mission(mainAppHandle)
            self.mainAppHandle = mainAppHandle;
        end

        %% Move Robot Function:
        function main(self, tm5Robot, tm12Robot, human, arm, products)
            % -------------- Sample path for first product -----------
            % --- Get on top of the product
            % Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  
            initalProductPose = products{1}.baseTr;

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.2; % adjust the height

            % Use RMRC to move the TM12 to on top of the product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle);

            % --- move to position to grasp product
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.05; % adjust the height

            % Use RMRC to move the TM12 to the product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle);

            % --- Move up with product
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.2; % adjust the height

            % Use RMRC to move the TM12 with product upward
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle, products{1});
            
            % --- Rotate the base pi
            newQ = tm12Robot.model.getpos();
            newQ = newQ(1) + pi;
            tm12Robot.AnimatePath(newQ,products{1});

            % --- Move the product to dropp off position
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            z = initalProductPose(3,4);
            desiredDropOff = [1.25, -0.5, z];

            % Use RMRC to place the product in desired position
            tm12Robot.rmrc(currentTM12Point, desiredDropOff, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle, products{1});

            % --- Use RMRC to move TM12 upward
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = desiredDropOff;
            desiredTM12Point(3) = desiredDropOff(3) + 0.2; % adjust the height

            % Use RMRC to move the TM12 to the product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle);

            %-------------- End of sample path ----------

            % This is 1 path
            % currentPose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
            % tm12EndPose = currentPose*transl(0.7, 0, 0);
            % 
            % currentPoint = currentPose(1:3,4)';
            % endPoint = tm12EndPose(1:3,4)';
            % 
            % tm12Robot.rmrc(currentPoint, endPoint, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle);
            % if (tm12Robot.obstacleAvoidance == true)
            %     self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
            %     disp('Fixing');
            % end
            % end of path

        end

        %% Fucntion to avoid collision
        function AvoidCollision(self, robot, endPose, human, arm, app)
            % Move up a little bit
            robot.avoidArmCheck = false;
            startPose = robot.model.fkine(robot.model.getpos()).T;
            endPoseTemp = startPose * transl(0,0,-0.3);
            startPose = startPose(1:3,4)';
            endPoseTemp = endPoseTemp(1:3,4)';
            robot.rmrc(startPose, endPoseTemp, robot.model.getpos(), 2, 10, human, arm, app);
            
            % Move to disired pose after moving up
            startPose = robot.model.fkine(robot.model.getpos()).T;
            startPose = startPose(1:3,4)';
            robot.rmrc(startPose, endPose, robot.model.getpos(), 2, 50, human, arm, app);

            robot.avoidArmCheck = true;

        end
        
        %% Function
        function testMain(self, tm5Robot, tm12Robot, human, arm)
            
            % Get the start pose:
            for i = 1:50
                startPose = tm12Robot.model.fkine(tm12Robot.model.getpos).T
                endPose = startPose*transl(0,0,0.2);
                startPoint = startPose(1:3,4)';
                endPoint = endPose(1:3,4)';
                tm12Robot.rmrc(startPoint, endPoint, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle);
                startPose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
                endPose = startPose*transl(0,0,-0.3);
                startPoint = startPose(1:3,4)';
                endPoint = endPose(1:3,4)';
                tm12Robot.rmrc(startPoint, endPoint, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle);
            end
       
        end

        %% Fucntion to generate path
        function waypointsT = GeneratePath(self, robot, endPose)
            startPose = robot.model.fkine(robot.model.getpos).T;
            startPoint = startPose(1:3,4)';
            endPoint = endPose(1:3,4)';
            steps = 50;
            waypoints = zeros(steps,3);
            % Generate path
            for i = 1:steps
                t = (i - 1) / (steps - 1);  % Interpolation parameter [0, 1]
                waypoints(i, :) = (1 - t) * startPoint + t * endPoint;
            end
            waypointsT = waypoints;
        end

        %% Function to move the two robots to homing position:
        function HomingRobot(self, tm5Robot, tm12Robot)
            
            % Define the homing position of the two robot:
            tm5qHome  = [pi, -pi/2, -pi/2, -pi/2, pi/2, 0];
            tm12qHome = [pi, -pi/2, -pi/2, -pi/2, pi/2, 0];
            
            % Create a trajectory for homing position
            path1 = jtraj(tm5Robot.model.getpos, tm5qHome, 50);
            path2 = jtraj(tm12Robot.model.getpos, tm12qHome, 50);

            for i = 1:50
                tm5Robot.model.animate(path1(i,:));
                tm12Robot.model.animate(path2(i,:));
                pause(0.05);
            end
        end


    end
end