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
        function main(self, tm5Robot, tm12Robot, human, arm)
            
            % This is 1 path
            currentPose = tm12Robot.model.fkine(tm12Robot.model.getpos).T
            tm12EndPose = currentPose*transl(0.7, 0, 0);
            % waypoints = self.GeneratePath(tm12Robot, tm12EndPose);
            % steps = 50;
            % for i=1:steps-1
            %     startPose = waypoints(i, :);
            %     endPose = waypoints(i+1, :);
            %     tm12Robot.rmrc(startPose, endPose, tm12Robot.model.getpos(), 0.8,2, human, arm, self.mainAppHandle);
            %     if (tm12Robot.obstacleAvoidance == true)
            %         self.AvoidCollision(tm12Robot, waypoints(steps,:));
            %         break;
            %     end
            % end

            currentPoint = currentPose(1:3,4)';
            endPoint = tm12EndPose(1:3,4)';

            tm12Robot.rmrc(currentPoint, endPoint, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle);
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end
            
            
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