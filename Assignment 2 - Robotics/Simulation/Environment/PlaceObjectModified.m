classdef PlaceObjectModified < handle    
    properties
        
        % Import File Name
        name;

        % Location:
        locations;

        % Plot:
        plot_h;

        % Colour:
        color;
        
        % Data of the object
        facesData;
        vertexData;
        data;
        vertexColours;
        vertices;
        transformedVertices;

        % Transformation of the object
        baseTr;

    end
    
    methods

        %% Constructor of this class:
        function self = PlaceObjectModified(filename, transform, colours)
            if nargin < 2
                transform = eye(4);
            end

            if nargin < 3
                colours = [0.5, 0.5, 0.5]; % Default colour is [0.5, 0.5, 0.5]
            end
            % Pass the input data to the data member of the class:
            self.baseTr = transform;
            self.name = filename;
            self.color = colours;
            
            % Set up the initial position of the object is at the origin:
            self.locations = [0,0,0];
            
            % Plot it:
            self.PlaceObject();
        end

        %% Plot and colour the object:
        function PlaceObject(self)
            % Read the input PLY file:
            [self.facesData, self.vertexData, self.data] = plyread(self.name, 'tri');
            
            % Scale the colours of the file to 0-to-1:
            try
                self.vertexColours = [self.data.vertex.red, self.data.vertex.green, self.data.vertex.blue] / 255;
            catch
                self.vertexColours = self.color/255;
            end
            % Plot the trisurf:
            self.plot_h = zeros(size(self.locations,1),1);

            for i = 1: size(self.locations, 1)
                self.plot_h(i) = trisurf(self.facesData,self.vertexData(:,1)+self.locations(i,1),self.vertexData(:,2)+self.locations(i,2), self.vertexData(:,3)+self.locations(i,3) ...
                    ,'FaceVertexCData',self.vertexColours,'EdgeColor','none','EdgeLighting','none');
            end

            % Get the vertices of the object:
            self.vertices = get(self.plot_h, 'Vertices');
            
            % Get the transformed vertices from the input transformation:
            self.transformedVertices = [self.vertices, ones(size(self.vertices, 1), 1)]*self.baseTr';
            set(self.plot_h, 'Vertices', self.transformedVertices(:,1:3));

            drawnow();

        end


        %% Move the object:
        function moveObject(self, transform)
            
            % Get the base transform:
            self.baseTr = transform;
            
            % Get the vertices of the object:
            self.transformedVertices = [self.vertices, ones(size(self.vertices, 1), 1)]*self.baseTr';
            set(self.plot_h, 'Vertices', self.transformedVertices(:,1:3));
            
            drawnow();
        end
        
    end
end

