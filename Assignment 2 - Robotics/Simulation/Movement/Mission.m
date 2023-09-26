classdef Mission < handle

    properties
        mainAppHandle = [];
    end

    methods
        
        %% Constructor of this class:
        function self = Mission(mainAppHandle)
            self.mainAppHandle = mainAppHandle;
        end

        %% Move Robot Function Step 1: TM12 move first product
        function MoveFirstProduct(self, tm5Robot, tm12Robot, human, arm, products)
            % -------------- Sample path for first product -----------
            % --- Get on top of the product
            % Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  
            initalProductPose = products.baseTr;

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.3; % adjust the height

            % Use RMRC to move the TM12 to on top of the product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, tm12Robot.model.getpos, 0.5, 50, human, arm, self.mainAppHandle);

            % --- move to position to grasp product
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.05; % adjust the height

            % Use RMRC to move the TM12 to the product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, tm12Robot.model.getpos, 0.5, 50, human, arm, self.mainAppHandle);

            % --- Move up with product
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.3; % adjust the height

            % Use RMRC to move the TM12 with product upward
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, tm12Robot.model.getpos, 0.5, 50, human, arm, self.mainAppHandle, products);
            
            % --- Rotate the base pi
            newQ = tm12Robot.model.getpos();
            newQ(1) = newQ(1) + pi;
            tm12Robot.AnimatePath(newQ,50,human,arm,self.mainAppHandle,products);

            % --- Move the product to above dropp off position
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            z = initalProductPose(3,4) + 0.3;
            desiredDropOff = [1.25, 0, z];

            % Use RMRC to place the product in above dropp off position
            tm12Robot.rmrc(currentTM12Point, desiredDropOff, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle, products);
            
            % --- Move the product to dropp off position
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            z = initalProductPose(3,4) + 0.05;
            desiredDropOff = [1.25, 0, z];

            % Use RMRC to place the product in above dropp off position
            tm12Robot.rmrc(currentTM12Point, desiredDropOff, tm12Robot.model.getpos, 2, 50, human, arm, self.mainAppHandle, products);

            % --- Use RMRC to move TM12 upward
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;  

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = desiredDropOff;
            desiredTM12Point(3) = desiredDropOff(3) + 0.3; % adjust the height

            % Use RMRC to move the TM12 to upward and ready for next product
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

        %% Function move robot step 2: 
        function FirstMoveBackAndFirstPick(self, tm5Robot, tm12Robot, human, arm, products)
            for k=2:6
                % -------------- Sample path for move back after first product and TM5 pick up -----------
                % --- Rotate the base pi
                newQ = tm12Robot.model.getpos();
                newQ(1) = newQ(1) - pi;
                tm12Robot.AnimatePath(newQ,50,human,arm,self.mainAppHandle);
                
                % --- Move TM5 approach the table
                if(k==2)
                    tm5Robot.MoveBase(0.75,0,0);            %CheckOk <!
                end
                
                % Homing TM5 before pick
                homeQTM5 = tm5Robot.qHomePick;
                tm5Robot.AnimatePath(homeQTM5, 50, human, arm, self.mainAppHandle);
    
                % Generate path for both TM12 and TM5 to move on top of the product 2 and 1 (move without product)
                initalProduct2Pose = products{k}.baseTr;
                tm12EndPose = initalProduct2Pose;
                tm12EndPose(3,4) = initalProduct2Pose(3,4) + 0.3;       % Fixed!
                waypointsTM12 = self.GeneratePath(tm12Robot, tm12EndPose);
                
                pickUpProductPose = products{k-1}.baseTr;
                tm5EndPose = pickUpProductPose;
                tm5EndPose(3,4) = pickUpProductPose(3,4) + 0.3;         % Fixed!
                waypointsTM5 = self.GeneratePath(tm5Robot, tm5EndPose);        
                
                % --- TM12 and TM5 move to on top of product 2 and product 1 (move without product)
                for i=1:49
                    startPoseTM12 = waypointsTM12(i, :);
                    endPoseTM12 = waypointsTM12(i+1, :);
    
                    startPoseTM5 = waypointsTM5(i,:);
                    endPoseTM5 = waypointsTM5(i+1,:);
    
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, tm12Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle);
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, tm5Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle);
                end             
                
                % Generate path for both TM12 and TM5 to pick up product 2 and 1 (move without product)
                tm12EndPose(3,4) = initalProduct2Pose(3,4) + 0.06;  % Fixed!
                waypointsTM12 = self.GeneratePath(tm12Robot, tm12EndPose);
    
                tm5EndPose(3,4) = pickUpProductPose(3,4) + 0.18; % Fixed!
                waypointsTM5 = self.GeneratePath(tm5Robot, tm5EndPose);
                
                % --- TM12 and TM5 move to pick up product 2 and product 1 (move without product)
                for i=1:49
                    startPoseTM12 = waypointsTM12(i, :);
                    endPoseTM12 = waypointsTM12(i+1, :);
    
                    startPoseTM5 = waypointsTM5(i,:);
                    endPoseTM5 = waypointsTM5(i+1,:);
    
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, tm12Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle);
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, tm5Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle);
                end
    
                % Generate path for both TM12 and TM5 move the product 2 and 1 up (move with product)
                tm12EndPose(3,4) = initalProduct2Pose(3,4) + 0.3;           %Fixed!
                waypointsTM12 = self.GeneratePath(tm12Robot, tm12EndPose);
    
                tm5EndPose(3,4) = pickUpProductPose(3,4) + 0.5;             %Fixed!
                waypointsTM5 = self.GeneratePath(tm5Robot, tm5EndPose);
    
                % --- TM12 and TM5 move product 2 and product 1 up (move with product)
                for i=1:49
                    startPoseTM12 = waypointsTM12(i, :);
                    endPoseTM12 = waypointsTM12(i+1, :);
    
                    startPoseTM5 = waypointsTM5(i,:);
                    endPoseTM5 = waypointsTM5(i+1,:);
    
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, tm12Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle, products{k});
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, tm5Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle, products{k-1});
                end
    
                % --- Rotate the base pi for both TM12 and TM5 (move with product)
    
                steps = 50;
    
                currentQTM5 = tm5Robot.model.getpos();
                newQTM5 = currentQTM5;           % Fixed!
                newQTM5(1) = currentQTM5(1) + pi;
                tm5Robot.AnimatePath(newQTM5, 50, human, arm, self.mainAppHandle, products{k-1});
                % pathTM5 = jtraj(currentQTM5, newQTM5, steps);
                
                currentQTM12 = tm12Robot.model.getpos();
                newQTM12 = currentQTM12;         % Fixed!
                newQTM12(1) = currentQTM12(1) + pi;
                % pathTM12 = jtraj(currentQTM12, newQTM12, steps);
                tm12Robot.AnimatePath(newQTM12, 50, human, arm, self.mainAppHandle, products{k});
    
                % for i=1:steps
                %     tm12Robot.AnimatePath(pathTM12(i,:), 1, human, arm, self.mainAppHandle, products{2});
                %     tm5Robot.AnimatePath(pathTM5(i,:), 1, human, arm, self.mainAppHandle, products{1});
                % end
                
                % --- TM12 and TM5 move product 2 and product 1 to above drop off position (move with product)
                zProduct2 = initalProduct2Pose(3,4) + 0.3;
                desiredDropOffTM12 = [1.25, 0, zProduct2];
                waypointsTM12 = self.GeneratePath(tm12Robot, transl(desiredDropOffTM12));
    
                baseTM5 = tm5Robot.model.base.T;
                zProduct1 = baseTM5(3,4) + 0.3;
                desiredDropOffTM5 = [0.25, 0, zProduct1];
                waypointsTM5 = self.GeneratePath(tm5Robot, transl(desiredDropOffTM5));
    
                % --- TM12 and TM5 move product 2 and product 1 to above drop off position (move with product)
                for i=1:49
                    startPoseTM12 = waypointsTM12(i, :);
                    endPoseTM12 = waypointsTM12(i+1, :);
    
                    startPoseTM5 = waypointsTM5(i,:);
                    endPoseTM5 = waypointsTM5(i+1,:);
    
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, tm12Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle, products{k});
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, tm5Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle, products{k-1});
                end
    
                % --- TM12 and TM5 move product 2 and product 1 to drop off position (move with product)
                zProduct2 = initalProduct2Pose(3,4) + 0.05;
                desiredDropOffTM12 = [1.25, 0, zProduct2];
                waypointsTM12 = self.GeneratePath(tm12Robot, transl(desiredDropOffTM12));
    
                baseTM5 = tm5Robot.model.base.T;
                zProduct1 = baseTM5(3,4) + 0.125;
                desiredDropOffTM5 = [0.25, 0, zProduct1];
                waypointsTM5 = self.GeneratePath(tm5Robot, transl(desiredDropOffTM5));
    
                % --- TM12 and TM5 move product 2 and product 1 to above drop off position (move with product)
                for i=1:49
                    startPoseTM12 = waypointsTM12(i, :);
                    endPoseTM12 = waypointsTM12(i+1, :);
    
                    startPoseTM5 = waypointsTM5(i,:);
                    endPoseTM5 = waypointsTM5(i+1,:);
    
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, tm12Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle, products{k});
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, tm5Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle, products{k-1});
                end
    
                % --- TM12 and TM5 move product 2 and product 1 back to above drop off position (move without product)
                zProduct2 = initalProduct2Pose(3,4) + 0.3;
                desiredDropOffTM12 = [1.25, -0.2, zProduct2];
                waypointsTM12 = self.GeneratePath(tm12Robot, transl(desiredDropOffTM12));
    
                baseTM5 = tm5Robot.model.base.T;
                zProduct1 = baseTM5(3,4) + 0.3;
                desiredDropOffTM5 = [0.25, 0, zProduct1];
                waypointsTM5 = self.GeneratePath(tm5Robot, transl(desiredDropOffTM5));
    
                % --- TM12 and TM5 move product 2 and product 1 back to above drop off position (move without product)
                for i=1:49
                    startPoseTM12 = waypointsTM12(i, :);
                    endPoseTM12 = waypointsTM12(i+1, :);
    
                    startPoseTM5 = waypointsTM5(i,:);
                    endPoseTM5 = waypointsTM5(i+1,:);
    
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, tm12Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle);
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, tm5Robot.model.getpos, 0.1, 2, human, arm, self.mainAppHandle);
                end
                %-------------- End of sample path ----------
            end
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