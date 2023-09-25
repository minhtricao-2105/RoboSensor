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
        function moveTM12ToPosition(self, robot, position, human)
            
            robot.rmrc(robot.model.fkine(robot.model.getpos).T, position, robot.model.getpos, human)

            % Update app position:
            self.mainAppHandle.UpdateJointStateData();
        end
    end
end