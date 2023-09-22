classdef Lab8Starter < handle
%#ok<*TRYNC>
%#ok<*NOPRT>
    properties
        %> Handle for the defined camera
        camera_h = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
            'resolution', [640 480], 'centre', [320 240], 'name', 'mycamera'); 

        %> Create 3D point(s) (V10 update: 'T' option replaced by 'pose'.)
        % gridOfSpheres = [0;0;5];
        % gridOfSpheres = [0.5,0;0,0.5;5,5];
        gridOfSpheres = mkgrid( 2, 0.5, 'pose', transl(0,0,5) );

        % Handle for the text to label the 1-based index of the points
        text_h;

        %> Error threshold (in pixels) for tracking (min and max)
        errorThreshold = [10,500];
        
        %> Max number of times to try to track an object
        maxAttempts = 50;
    end

    methods
        function self = Lab8Starter()            
            close all          
            % Camera view and plotting
            self.camera_h.clf()

            % Set the position of the camera
            self.UpdateCameraAndView( transl(0,0,1) * troty(0.1) )
            
            plot_sphere(self.gridOfSpheres, 0.05, 'b')
            grid on
            self.AddCameraControls();
        end

        %% UpdateCameraAndView
        function uv = UpdateCameraAndView(self,tr)
            if 1 < nargin 
                self.camera_h.T = tr;               
            end

            % Update where the camera is plotted
            self.camera_h.plot_camera(self.gridOfSpheres, 'label', 'scale', 0.15);
                
            % Update what the camera sees
            uv = self.camera_h.plot(self.gridOfSpheres, 'o');
           
            for i = 1:size(uv,2)
                try delete(self.text_h(i)); end 
                self.text_h(i) = text(uv(1,i), uv(2,i), sprintf('%d', i), 'Color', 'red', 'FontSize', 12,'Parent',gca(self.camera_h.figure));
            end
            drawnow
        end

        %% AddCameraControls
        function AddCameraControls(self)
            sliderProperties = struct('Style', 'slider', 'Value', 0, 'SliderStep', [0.01 0.2], 'Callback', @self.SliderCallback);
            uicontrol(sliderProperties ,  'Position', [0 60 200 10], 'String', 'sliderX' , 'Min', -1 ,  'Max', 3);
            uicontrol(sliderProperties ,  'Position', [0 50 200 10], 'String', 'sliderY' , 'Min', -1 ,  'Max', 3);
            uicontrol(sliderProperties ,  'Position', [0 40 200 10], 'String', 'sliderZ' , 'Min', -1 ,  'Max', 3);
            uicontrol(sliderProperties,   'Position', [0 20 200 10], 'String','sliderRotY','Min', -pi/4,'Max', pi/4);

            uicontrol('Style','pushbutton','String','Set Target','Position', [0 20 100 20],'Parent',self.camera_h.figure,'Callback', @self.TargetUserSelectedPointsWithImagejacobian);
        end
        
        %% SliderCallback
        function SliderCallback(self,source, ~)
            sliderValue = get(source, 'Value');
            tr = self.camera_h.T.T;

            switch source.String
                case 'sliderX'
                    tr(1,4) = sliderValue;
                case 'sliderY'
                    tr(2,4) = sliderValue;
                case 'sliderZ'
                    tr(3,4) = sliderValue;
                case 'sliderRotY'
                    tr(1:3,1:3) = roty(sliderValue);
            end           
            self.UpdateCameraAndView(tr);
        end      

        %% TargetUserSelectedPointsWithImagejacobian
        function TargetUserSelectedPointsWithImagejacobian(self,source,~) 
            source.Visible = false;
            disp('Click on the figure where you want the updated point(s) to be in order, then press Enter')
            [uStar,vStar] = getpts(self.camera_h.figure) 

            uv = self.UpdateCameraAndView;

            errorFeatures = [uStar';vStar'] - uv;
            errorFeatures = errorFeatures(:);
            previousErrorFeatures = errorFeatures;
            attempts = 0;
            
            % Keep going until the error is too small or large
            while self.errorThreshold(1) < norm(errorFeatures) ...
                && norm(errorFeatures) < self.errorThreshold(2)

                tr = self.camera_h.T.T;    
                cameraToCenterDistance = norm( tr(1:3,4) - mean(self.gridOfSpheres,2) );    
                J = self.camera_h.visjac_p(uv,cameraToCenterDistance);

                %% Gain of the controler (should be a class property, but put here to explain)
                lambda = 0.1; % If you make it too high (1 or above) it may get there in 1 step or overshoot too much and loose track

                deltaX = lambda * pinv(J) * errorFeatures;


                %% Play here with only allowing certain movements
                tr(1:3,4) = tr(1:3,4) + deltaX(1:3);
                tr(1:3,1:3) = tr(1:3,1:3) * rotx(deltaX(4)) * roty(deltaX(5)) * rotz(deltaX(6));
                % tr(1:3,1:3) = tr(1:3,1:3) * roty(deltaX(5))
              
                uv = self.UpdateCameraAndView(tr);
                errorFeatures = [uStar';vStar'] - uv
                errorFeatures = errorFeatures(:)

                % Is it both less than max attempts and reducing the error?
                if (attempts < self.maxAttempts)
                    attempts = attempts + 1
                    if (norm(previousErrorFeatures) < norm(errorFeatures))
                        disp('Error is increasing');
                    end
                else
                    disp('Cannot get closer: breaking out to prevent infinitely tracking an impossible target')
                    break;
                end
                previousErrorFeatures = errorFeatures;
            end            

            if (isnan(errorFeatures))
                error('An impossible situtation has occured. Please reinstantiate the class and more carefully choose the targets')
            end

            source.Visible = true;
        end
    end
end