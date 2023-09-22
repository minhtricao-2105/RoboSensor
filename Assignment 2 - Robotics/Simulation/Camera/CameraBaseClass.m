classdef CameraBaseClass < handle

    properties

        % --- Get the handle for the defined camera:
        camera_h;

        % --- Get the transformation between the eeTr and the camera
        cameraEE = transl(0, -0.06, -0.01);

        % -- Create 3D points
        gridOfSpheres = mkgrid( 2, 0.5, 'pose', transl(0,0,5) );
    end

    methods

        %% Constructor of this class:
        function self =  CameraBaseClass(transform)

            % Create a camera:
            self.camera_h = CentralCamera('focal', 0.015, 'pixel', 10e-6, ...
                'resolution', [640 480], 'centre', [320 240], 'name', 'mycamera');

            % Cameara view and plotting:
            self.camera_h.clf()

            % Set the position of the camera:
            self.moveCamera(transform);
        end

        %% UpdateCameraAndView:
        function moveCamera(self, transform, points)
            if 1 < nargin
                self.camera_h.T = transform*self.cameraEE;
            end
            
            % Update where the camera is plotted
            self.camera_h.plot_camera('scale', 0.05, 'color', 'k');
            
            drawnow
        end

        %% Update Camera View:


    end
end