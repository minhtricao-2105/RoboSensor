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

            % --- I. Get on top of the product
            % WayPoint1: Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
            initalProductPose = products.baseTr;

            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.3; % adjust the height

            % Use RMRC to move the TM12 to on top of the product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, 0.25, 40, human, arm, self.mainAppHandle);

            % Apply avoid collision:
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- II. Move the TM12 to the pick up position:
            % WayPoint2: Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.05; % adjust the height

            % Use RMRC to move the TM12 to the product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, 0.25, 40, human, arm, self.mainAppHandle);

            % Apply avoid collision:
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- III. Move the product up
            % WayPoint3: Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = initalProductPose(1:3,4)';
            desiredTM12Point(3) = desiredTM12Point(3) + 0.3; % adjust the height

            % Use RMRC to move the TM12 with product upward
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, 0.25, 40, human, arm, self.mainAppHandle, products);

            % Apply avoid collision:
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- IV. Rotate the base by 180 degrees:
            newQ = tm12Robot.model.getpos();
            newQ(1) = newQ(1) + pi;
            tm12Robot.AnimatePath(newQ,50,human,arm,self.mainAppHandle,products);

            % Apply avoid collision:
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- V. Move the product to above drop off position
            % WayPoint4: Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
            currentTM12Point = currentTM12Pose(1:3,4)';
            z = initalProductPose(3,4) + 0.3;
            desiredDropOff = [1.25, 0, z];

            % Use RMRC to place the product in above dropp off position
            tm12Robot.rmrc(currentTM12Point, desiredDropOff, 0.25, 40, human, arm, self.mainAppHandle, products);

            % Apply avoid collision:
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- VI. Move the product to the drop off position
            % WayPoint5: Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
            currentTM12Point = currentTM12Pose(1:3,4)';
            z = initalProductPose(3,4) + 0.05;
            desiredDropOff = [1.25, 0, z];

            % Use RMRC to place the product in above dropp off position
            tm12Robot.rmrc(currentTM12Point, desiredDropOff, 0.25, 40, human, arm, self.mainAppHandle, products);

            % Apply avoid collision:
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- VII. Move the TM12 upward after drooping the product:
            % WayPoint6: Define start point and end point and use it for RMRC
            currentTM12Pose = tm12Robot.model.fkine(tm12Robot.model.getpos).T;
            currentTM12Point = currentTM12Pose(1:3,4)';
            desiredTM12Point = desiredDropOff;
            desiredTM12Point(3) = desiredDropOff(3) + 0.3; % adjust the height

            % Use RMRC to move the TM12 to upward and ready for next product
            tm12Robot.rmrc(currentTM12Point, desiredTM12Point, 0.25, 40, human, arm, self.mainAppHandle);

            % Apply avoid collision:
            if (tm12Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm12Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

        end

        %% Function move robot step 2:
        function FirstMoveBackAndFirstPick(self, tm5Robot, tm12Robot, human, arm, products)

            for k=2:6
                % -------------- Sample path for move back after first product and TM5 pick up -----------
                % --- Rotate the base of the TM12 by 180 degree
                newQ = tm12Robot.model.getpos();
                newQ(1) = newQ(1) - pi;
                tm12Robot.AnimatePath(newQ,50,human,arm,self.mainAppHandle);

                % --- Move TM5 approach the table
                if(k == 2)
                    tm5Robot.MoveBase(0.75,0,0, human, self.mainAppHandle);
                end

                % --- Homing the TM5 Robot before picking the brick:
                homeQTM5 = tm5Robot.qHomePick;
                tm5Robot.AnimatePath(homeQTM5, 50, human, arm, self.mainAppHandle);

                % --- I. Generate path for both TM12 and TM5 to move on top of the product 2 and 1 (move without product)
                % WayPoint 1: On Top of Product Picking:

                % - Path for the TM12 Robot to the waypoint above the product:
                initalProduct2Pose = products{k}.baseTr;
                tm12EndPose = initalProduct2Pose;
                tm12EndPose(3,4) = initalProduct2Pose(3,4) + 0.3;
                waypointsTM12 = self.GeneratePath(tm12Robot, tm12EndPose);

                % - Path for the TM5 Robot to the waypoint above the product
                pickUpProductPose = products{k-1}.baseTr;
                tm5EndPose = pickUpProductPose;
                tm5EndPose(3,4) = pickUpProductPose(3,4) + 0.3;
                waypointsTM5 = self.GeneratePath(tm5Robot, tm5EndPose);

                % --- TM12 and TM5 move to on top of product 2 and product 1 (move without product)
                % - Begin to perform the path by RMRC motion:
                self.SimulateTwoRobot(tm5Robot, tm12Robot, waypointsTM5, waypointsTM12, human, arm, k);

                % --- II. Generate path for both TM12 and TM5 to pick up product 2 and 1 (move without product)
                % WayPoint2: To The Product Picking

                % - Path for the TM12 Robot to the pick the product:
                tm12EndPose(3,4) = initalProduct2Pose(3,4) + 0.06;
                waypointsTM12 = self.GeneratePath(tm12Robot, tm12EndPose);

                % - Path for the TM15 Robot to the pick the product:
                tm5EndPose(3,4) = pickUpProductPose(3,4) + 0.18;
                waypointsTM5 = self.GeneratePath(tm5Robot, tm5EndPose);

                % --- TM12 and TM5 move to pick up product 2 and product 1 (move without product)

                % - Begin to perform the path by RMRC motion to pick up products
                self.SimulateTwoRobot(tm5Robot, tm12Robot, waypointsTM5, waypointsTM12, human, arm, k);

                % --- III . Generate path for both TM12 and TM5 move the product 2 and 1 up (move with product)
                % WayPoint3: Move Product Up Picking

                % - Path for the TM12 Robot to move the product up:
                tm12EndPose(3,4) = initalProduct2Pose(3,4) + 0.3;
                waypointsTM12 = self.GeneratePath(tm12Robot, tm12EndPose);

                % - Path for the TM5 Robot to move the product up:
                tm5EndPose(3,4) = pickUpProductPose(3,4) + 0.5;
                waypointsTM5 = self.GeneratePath(tm5Robot, tm5EndPose);

                % --- TM12 and TM5 move product 2 and product 1 up (move with product)
                self.SimulateTwoRobot(tm5Robot, tm12Robot, waypointsTM5, waypointsTM12, human, arm, k, products);

                % --- IV. Rotate the base by 180 degree for both TM12 and TM5 (move with product)
                % WayPoint4: Rotate the Base

                % - Generate Path and Rotate the TM5:
                currentQTM5 = tm5Robot.model.getpos();
                newQTM5 = currentQTM5;
                newQTM5(1) = currentQTM5(1) - pi;
                tm5Robot.AnimatePath(newQTM5, 50, human, arm, self.mainAppHandle, products{k-1});

                % - Generate Path and Rotate the TM12
                currentQTM12 = tm12Robot.model.getpos();
                newQTM12 = currentQTM12;
                newQTM12(1) = currentQTM12(1) + pi;
                tm12Robot.AnimatePath(newQTM12, 50, human, arm, self.mainAppHandle, products{k});

                % --- V. TM12 and TM5 move product 2 and product 1 to above drop off position (move with product)
                % WayPoint5: Above the Product Dropping

                % - Generate Path for the TM12 to the above position before dropping off the product:
                zProduct2 = initalProduct2Pose(3,4) + 0.3;
                desiredDropOffTM12 = [1.25, 0, zProduct2];
                waypointsTM12 = self.GeneratePath(tm12Robot, transl(desiredDropOffTM12));

                % - Generate Path for the TM5 to the above position before dropping off the product:
                baseTM5 = tm5Robot.model.base.T;
                zProduct1 = baseTM5(3,4) + 0.4;
                desiredDropOffTM5 = [0.1, -0.1, zProduct1];
                if (k == 2) || (k == 5)
                    desiredDropOffTM5 = [0.1, -0.1, zProduct1];
                elseif (k == 3) || (k == 6)
                    desiredDropOffTM5 = [0.1, 0.1, zProduct1];
                elseif (k == 4)
                    desiredDropOffTM5 = [0.35, 0, zProduct1];
                end
                waypointsTM5 = self.GeneratePath(tm5Robot, transl(desiredDropOffTM5));

                % --- TM12 and TM5 move product 2 and product 1 to above drop off position (move with product)
                self.SimulateTwoRobot(tm5Robot, tm12Robot, waypointsTM5, waypointsTM12, human, arm, k, products);

                % --- VI. TM12 and TM5 move product 2 and product 1 to drop off position (move with product)
                % WayPoint6: To The Product Dropping

                % - Generate Path for the TM12 to drop off the product:
                zProduct2 = initalProduct2Pose(3,4) + 0.05;
                desiredDropOffTM12 = [1.25, 0, zProduct2];
                waypointsTM12 = self.GeneratePath(tm12Robot, transl(desiredDropOffTM12));

                % - Generate Path for the TM5 to drop off the product:
                baseTM5 = tm5Robot.model.base.T;
                if (k == 2) || (k == 3) || (k == 4)
                    zProduct1 = baseTM5(3,4) + 0.2;
                elseif (k == 5) || (k == 6)
                    zProduct1 = baseTM5(3,4) + 0.25;
                end
                desiredDropOffTM5 = [0.1, -0.1, zProduct1];
                if (k == 2) || (k == 5)
                    desiredDropOffTM5 = [0.1, -0.1, zProduct1];
                elseif (k == 3) || (k == 6)
                    desiredDropOffTM5 = [0.1, 0.1, zProduct1];
                elseif (k == 4)
                    desiredDropOffTM5 = [0.35, 0, zProduct1];
                end
                waypointsTM5 = self.GeneratePath(tm5Robot, transl(desiredDropOffTM5));

                % --- TM12 and TM5 move product 2 and product 1 to above drop off position (move with product)
                self.SimulateTwoRobot(tm5Robot, tm12Robot, waypointsTM5, waypointsTM12, human, arm, k, products);

                % --- VI. TM12 and TM5 move up after dropping products
                % WayPoint7: Moving Up before Homing: % - Path for the TM12 Robot to the pick the product:

                % - Generate Path for the TM12 to move up before homing
                zProduct2 = initalProduct2Pose(3,4) + 0.3;
                desiredDropOffTM12 = [1.25, -0.2, zProduct2];
                waypointsTM12 = self.GeneratePath(tm12Robot, transl(desiredDropOffTM12));

                % - Generate Path for the TM5 to move up before homing
                desiredPoseTM5 = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
                desiredPoseTM5(3,4) = desiredPoseTM5(3,4) + 0.2;
                waypointsTM5 = self.GeneratePath(tm5Robot, desiredPoseTM5);

                % --- TM12 and TM5 move product 2 and product 1 back to above drop off position (move without product)
                self.SimulateTwoRobot(tm5Robot, tm12Robot, waypointsTM5, waypointsTM12, human, arm, k);
                
                %-------------- End of sample path ----------
            end
        end

        %% Function robot move step 3
        function PickLastProduct(self, tm5Robot, tm12Robot, human, arm, products)
            % -------------- Sample path for last product -----------
            tm12qHome = [0, -pi/2, -pi/2, -pi/2, pi/2, 0];
            tm12Robot.AnimatePath(tm12qHome, 50, human, arm, self.mainAppHandle);

            % --- Homing the TM5 Robot before picking the brick:
            homeQTM5 = tm5Robot.qHomePick;
            tm5Robot.AnimatePath(homeQTM5, 50, human, arm, self.mainAppHandle);
            
            % --- I. Get on top of the last product (move without product)
            % WayPoint1: Define start point and end point and use it for RMRC
            currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            initalProductPose = products.baseTr;

            currentTM5Point = currentTM5Pose(1:3,4)';
            desiredTM5Point = initalProductPose(1:3,4)';
            desiredTM5Point(3) = desiredTM5Point(3) + 0.3; % adjust the height

            % Use RMRC to move the TM5 to on top of the product
            tm5Robot.rmrc(currentTM5Point, desiredTM5Point, 0.25, 40, human, arm, self.mainAppHandle);

            % Apply avoid collision:
            if (tm5Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm5Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- II. Move the TM5 to the pick up position (move without product)
            % WayPoint2: Define start point and end point and use it for RMRC
            currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            currentTM5Point = currentTM5Pose(1:3,4)';
            desiredTM5Point = initalProductPose(1:3,4)';
            desiredTM5Point(3) = desiredTM5Point(3) + 0.18; % adjust the height

            % Use RMRC to move the TM12 to the product
            tm5Robot.rmrc(currentTM5Point, desiredTM5Point, 0.25, 40, human, arm, self.mainAppHandle);

            % Apply avoid collision:
            if (tm5Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm5Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- III. Move the product up (move with porduct)
            % WayPoint3: Define start point and end point and use it for RMRC
            currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            currentTM5Point = currentTM5Pose(1:3,4)';
            desiredTM5Point = initalProductPose(1:3,4)';
            desiredTM5Point(3) = desiredTM5Point(3) + 0.5; % adjust the height

            % Use RMRC to move the TM12 with product upward
            tm5Robot.rmrc(currentTM5Point, desiredTM5Point, 0.25, 40, human, arm, self.mainAppHandle, products);

            % Apply avoid collision:
            if (tm5Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm5Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- IV. Rotate the base by 180 degrees:
            newQ = tm5Robot.model.getpos();
            newQ(1) = newQ(1) - pi;
            tm5Robot.AnimatePath(newQ,50,human,arm,self.mainAppHandle,products);

            % Apply avoid collision:
            if (tm5Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm5Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- V. Move the product to above drop off position
            % WayPoint4: Define start point and end point and use it for RMRC
            currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            currentTM5Point = currentTM5Pose(1:3,4)';
            baseTM5 = tm5Robot.model.base.T;
            z = baseTM5(3,4) + 0.4;
            desiredDropOff = [0.35, 0, z];

            % Use RMRC to place the product in above dropp off position
            tm5Robot.rmrc(currentTM5Point, desiredDropOff, 0.25, 40, human, arm, self.mainAppHandle, products);

            % Apply avoid collision:
            if (tm5Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm5Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- VI. Move the product to the drop off position
            % WayPoint5: Define start point and end point and use it for RMRC
            currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            currentTM5Point = currentTM5Pose(1:3,4)';
            baseTM5 = tm5Robot.model.base.T;
            z = baseTM5(3,4) + 0.25;
            desiredDropOff = [0.35, 0, z];

            % Use RMRC to place the product in above dropp off position
            tm5Robot.rmrc(currentTM5Point, desiredDropOff, 0.25, 40, human, arm, self.mainAppHandle, products);

            % Apply avoid collision:
            if (tm5Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm5Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % --- VII. Move the TM12 upward after drooping the product:
            % WayPoint6: Define start point and end point and use it for RMRC
            currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            currentTM5Point = currentTM5Pose(1:3,4)';
            desiredTM5Point = desiredDropOff;
            desiredTM5Point(3) = desiredDropOff(3) + 0.3; % adjust the height

            % Use RMRC to move the TM12 to upward and ready for next product
            tm5Robot.rmrc(currentTM5Point, desiredTM5Point, 0.25, 40, human, arm, self.mainAppHandle);

            % Apply avoid collision:
            if (tm5Robot.obstacleAvoidance == true)
                self.AvoidCollision(tm5Robot, endPoint, human, arm, self.mainAppHandle);
                disp('Fixing');
            end

            % -------------- End of path ---------------
        end

        %% Function move TM5 to fill the shelves
        function TM5FillProduct(self, tm5Robot, tm12Robot, human, arm, products)
            % -------------- Sample path for deliver product -----------

            % --- I. Deliver red product location
            tm5Robot.MoveBase(-0.75, -1.8, 0, human, self.mainAppHandle, products);
            tm5Robot.MoveBase(0, 0, -90, human, self.mainAppHandle, products);
            
            for i=1:6
                if (i == 1)
                    k = 4;
                elseif (i == 2)
                    k = 1;
                elseif (i == 3)
                    k = 5;
                elseif (i == 4)
                    k = 2;
                elseif (i == 5)
                    k = 6;
                elseif (i == 6)
                    k = 3;
                end
                % --- I. Adjust Q and move base 
                tm5Robot.AnimatePath(tm5Robot.qHomePlace,50,human,arm,self.mainAppHandle);
                if (i == 3)
                    prod1 = products{5};
                    prod2 = products{2};
                    prod3 = products{6};
                    prod4 = products{3};

                    remainProds = {prod1, prod2, prod3, prod4};

                    tm5Robot.MoveBase(0, -1.03, 0, human, self.mainAppHandle, remainProds);
                elseif (i == 5)
                    prod1 = products{6};
                    prod2 = products{3};

                    remainProds = {prod1, prod2};

                    tm5Robot.MoveBase(0, -1.03, 0, human, self.mainAppHandle, remainProds);
                end
                
                % --- II. Go to above position of red product (move without product)
                currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
                pickUpPose = products{k}.baseTr;
    
                currentTM5Point = currentTM5Pose(1:3,4)';
                desiredTM5Point = pickUpPose(1:3,4)';
                desiredTM5Point(3) = desiredTM5Point(3) + 0.3;
    
                % Use RMRC to move the TM5 to the product
                tm5Robot.rmrc(currentTM5Point, desiredTM5Point, 0.25, 40, human, arm, self.mainAppHandle);
    
                % --- III. Go to pick up red product (move without product)
                currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
                currentTM5Point = currentTM5Pose(1:3,4)';
    
                desiredTM5Point = pickUpPose(1:3,4)';
                desiredTM5Point(3) = desiredTM5Point(3) + 0.18;
    
                % Use RMRC to move the TM5 to on top of the product
                tm5Robot.rmrc(currentTM5Point, desiredTM5Point, 0.25, 40, human, arm, self.mainAppHandle);
    
                % --- IV. Go back to above position of the red product (move product)
                currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
                currentTM5Point = currentTM5Pose(1:3,4)';
                
                desiredTM5Point = pickUpPose(1:3,4)';
                desiredTM5Point(3) = desiredTM5Point(3) + 0.3;
    
                % Use RMRC to move the TM5 with the product upward
                tm5Robot.rmrc(currentTM5Point, desiredTM5Point, 0.25, 40, human, arm, self.mainAppHandle, products{k});
    
                % --- V. Rotate the base by 180 degrees:
                % newQ = tm5Robot.model.getpos();
                % newQ(1) = newQ(1) - pi;
                % tm5Robot.AnimatePath(newQ,50,human,arm,self.mainAppHandle,products{4});
    
                homeQTM5 = tm5Robot.qHomePick;
                tm5Robot.AnimatePath(homeQTM5, 50, human, arm, self.mainAppHandle, products{k});
    
                % --- VI. Adjust the orientation of the product
                newQ = tm5Robot.model.getpos();
                newQ(4) = newQ(4) + deg2rad(30);
                tm5Robot.AnimatePath(newQ,50,human,arm,self.mainAppHandle,products{k});
    
                % --- VII. Move to above desired location
                currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
                currentTM5Point = currentTM5Pose(1:3,4)';
    
                baseTM5 = tm5Robot.model.base.T;
                z = baseTM5(3,4) + 0.6;
                x = baseTM5(1,4) + 0.1;
                y = baseTM5(2,4) - 0.626;
                desiredDropOff = [x, y, z];
    
                % Use RMRC to place the product in above dropp off position
                tm5Robot.rmrc(currentTM5Point, desiredDropOff, 0.25, 40, human, arm, self.mainAppHandle, products{k});
    
                % --- VIII. Move to above desired location
                currentTM5Pose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
                currentTM5Point = currentTM5Pose(1:3,4)';
    
                baseTM5 = tm5Robot.model.base.T;
                z = baseTM5(3,4) + 0.35;
                if (k == 1) || (k == 2) || (k == 3)
                    x = baseTM5(1,4) - 0.1;
                else
                    x = baseTM5(1,4) + 0.1;
                end
                y = baseTM5(2,4) - 0.626;
                desiredDropOff = [x, y, z];
    
                % Use RMRC to place the product in above dropp off position
                tm5Robot.rmrc(currentTM5Point, desiredDropOff, 0.25, 40, human, arm, self.mainAppHandle, products{k});
            end
            % -------------- End of path ---------------
        end

        %% Homing everything
        function HomeTM5(self, tm5Robot)
            
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
            tm5Robot.MoveBase(0.75,0,0, human, self.mainAppHandle);

            startPose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            startPoint = startPose(1:3,4)';
            desiredDropOffTM5 = [1.25, 0, 0.95];
            tm5Robot.rmrc(startPoint, desiredDropOffTM5, 2, 50, human, arm, self.mainAppHandle);

            startPose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            startPoint = startPose(1:3,4)';
            desiredDropOffTM5 = [1.25, 0, 1.25];
            tm5Robot.rmrc(startPoint, desiredDropOffTM5, 2, 50, human, arm, self.mainAppHandle);

            currentQ = tm5Robot.model.getpos();
            newQ = currentQ;
            newQ(1) = currentQ(1) - pi;
            tm5Robot.AnimatePath(newQ, 50, human, arm, self.mainAppHandle);

            startPose = tm5Robot.model.fkine(tm5Robot.model.getpos).T;
            startPoint = startPose(1:3,4)';
            baseTM5 = tm5Robot.model.base.T;
            zProduct1 = baseTM5(3,4) + 0.1;
            desiredDropOffTM5 = startPoint;
            desiredDropOffTM5(3) = zProduct1;
            tm5Robot.rmrc(startPoint, desiredDropOffTM5, 2, 50, human, arm, self.mainAppHandle);

        end

        %% Fucntion to generate path
        function waypointsT = GeneratePath(self, robot, endPose)
            startPose = robot.model.fkine(robot.model.getpos).T;
            startPoint = startPose(1:3,4)';
            endPoint = endPose(1:3,4)';
            steps = 25;
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
        
        %% Function simulate two robot
        function SimulateTwoRobot(self, tm5Robot, tm12Robot, waypointsTM5, waypointsTM12, human, arm, index, products)
            moveProduct = true;
            
            if nargin < 9
                moveProduct = false;
            end
            
            % Begin the for loop:
            for i=1:24

                % - Path Belong to the TM12
                startPoseTM12 = waypointsTM12(i, :);
                endPoseTM12 = waypointsTM12(i+1, :);

                % - Path Belong to the TM5
                startPoseTM5 = waypointsTM5(i,:);
                endPoseTM5 = waypointsTM5(i+1,:);

                % - Perform RMRC:
                if moveProduct == false
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, 0.1, 2, human, arm, self.mainAppHandle);
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, 0.1, 2, human, arm, self.mainAppHandle);
                else
                    tm12Robot.rmrc(startPoseTM12, endPoseTM12, 0.1, 2, human, arm, self.mainAppHandle, products{index});
                    tm5Robot.rmrc(startPoseTM5, endPoseTM5, 0.1, 2, human, arm, self.mainAppHandle, products{index-1});
                end

            end

        end

        %% Function to test fill
        function TestFillProduct(self, tm5Robot, products)
            tm5Robot.TestMoveBase(0.75,0,0);

            for k=1:6
                baseTM5 = tm5Robot.model.base.T;
                if (k == 1) || (k == 2) || (k == 3)
                    zProduct1 = baseTM5(3,4) + 0.05;
                elseif (k == 5) || (k == 6) || (k == 4)
                    zProduct1 = baseTM5(3,4) + 0.1;
                end
                position = [0.1, -0.1, zProduct1];
                if (k == 1) || (k == 4)
                    x = baseTM5(1,4) - 0.65;
                    y = baseTM5(2,4) - 0.1;
                    position = [x, y, zProduct1];
                elseif (k == 2) || (k == 5)
                    x = baseTM5(1,4) - 0.65;
                    y = baseTM5(2,4) + 0.1;
                    position = [x, y, zProduct1];
                elseif (k == 3) || (k == 6)
                    x = baseTM5(1,4) - 0.4;
                    y = baseTM5(2,4);
                    position = [x, y, zProduct1];    
                end

                products{k}.moveObject(transl(position));

            end
        end

    end
end