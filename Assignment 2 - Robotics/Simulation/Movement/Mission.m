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
            
            startPose = tm12Robot.model.fkine(tm12Robot.model.getpos()).T;
            endPose = startPose*transl(0.5, 0, 0);
            ttRobot.rmrc(startPose, endPose, tm12Robot.model.getpos(), human);

        end
    end
end