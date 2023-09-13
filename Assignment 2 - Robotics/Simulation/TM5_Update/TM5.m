classdef TM5 < BaseClass
    %% UR3 Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'TM5';
    end
    
    methods
%% Constructor
        function self = TM5(baseTr)
            % Create the model of the UR3 robot  
			self.CreateModel();
            % if there is no special input for base, default value will be applied (=4x4 identity matrix)
            if nargin < 1			
				baseTr = eye(4);				
            end
            % The base of the UR3 robot is based on the base input
            self.model.base = self.model.base.T * baseTr;
            
            % Plot the UR3 model on simulation environment
            self.PlotAndColourRobot(); 
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.146,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.329,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.3115,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.109,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            % link(5) = Link('d',0.106,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            % link(6) = Link('d',0.1132,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
