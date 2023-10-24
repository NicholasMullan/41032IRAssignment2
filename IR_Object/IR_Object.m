% Based on the brick class from Assignment 1. Updated to be generalised to
% any object. 

classdef IR_Object < handle
    properties
        BasePose
        Moved = false
        Name
        workspace = [-1.8 1.8 -1.8 1.8 -0.3 2];
        model
        Type
        Color % Add a Color property to store the object's color
    end

    methods
        function self = IR_Object(type, name, pose, color)
            self.Name = name;
            self.BasePose = pose;
            self.Type = type;
            self.Color = color; % Set the Color property
            % Load the color map
            colorMap = containers.Map;
            colorMap('Coca-Cola') = [1, 0, 0];
            colorMap('Fanta') = [0, 1, 0];
            colorMap('Lemonade') = [0, 0, 1];
            colorMap('Solo') = [1, 1, 0];
            % Add more color mappings as needed

            [faceData, vertexData, colorName] = plyread(self.Type, 'tri');
            link1 = Link('alpha', 0, 'a', 0, 'd', 0, 'offset', 0);
            self.model = SerialLink(link1, 'name', name);
            self.model.faces = {[], faceData};
            self.model.points = {[], vertexData};
            self.model.base = pose;
            self.model.base = self.model.base.T;

            hold on;
            % Set the color based on the color mapping
            if isKey(colorMap, colorName)
                rgb = colorMap(colorName);
                set(self.model, 'rgb', rgb);
            end

            plot3d(self.model, 0, 'workspace', self.workspace, 'view', [-30, 30], 'delay', 0, 'noarrow', 'nowrist', 'rgb', self.Color, 'plotopt', {'noname', 'flat'});
        end

        function UpdatePose(self, newPose)
            self.BasePose = newPose;
            % Load vertices and faces from the PLY file
            [faceData, vertexData, colorName] = plyread(self.Type, 'tri');
            
            % Set the color based on the color mapping
            if isKey(colorMap, colorName)
                rgb = colorMap(colorName);
                set(self.model, 'rgb', rgb);
            end

            % Transform vertices according to the new pose
            transformedVertices = (self.BasePose * [vertexData(:, 1:3), ones(size(vertexData, 1), 1)]')';
            
            % Update the object's visual representation with the correct color
            set(self.model, 'Vertices', transformedVertices(:, 1:3));
        end

        function MarkAsMoved(self)
            self.Moved = true;
        end
    end
end
