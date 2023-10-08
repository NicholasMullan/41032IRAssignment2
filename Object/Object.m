% Based on the brick class from Assignment 1. Updated to be generalised to
% any object. 


classdef Object < handle

    properties
        
        BasePose %Hold the value of the base pose
        BaseOrientation %Hold the value of the base orientation
        Moved = false % Change to true when move has been completed
        Colour
        Name
        workspace = [-1.8 1.8 -1.8 1.8 -0.3 2];
        model
    end

    methods 
        %Overload constructor to allow for creating an object type with
        %string input, the name of it, the pose and a colour override in
        %testing. 
        %Note the type MUST include .ply in the name
        
        function self = Object(type, name, pose, colour)
        self.Name = name;
        self.BasePose = pose;
        self.BaseOrientation = orientation;

         % Load vertices and faces from the PLY file
        [faceData, vertexData] = plyread(type, 'tri');
        link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
        self.model = SerialLink(link1,'name',name);
        self.model.faces = {[], faceData};
        self.model.points = {[], vertexData};
        self.model.base = pose;
        self.model.base = self.model.base.T * trotz(pi/2);

        hold on;

        plot3d(self.model, 0, 'workspace',self.workspace,'view',[-30,30],'delay',0,'noarrow','nowrist');
        end

        
       function UpdatePose(self, newPose)
            self.BasePose = newPose;
            
            % Load vertices and faces from the PLY file
            [faceData, vertexData] = plyread('HalfSizedRedGreenBrick.ply', 'tri');
            
            % Transform vertices according to the new pose
            transformedVertices = (self.BasePose * [vertexData(:, 1:3), ones(size(vertexData, 1), 1)]')';
            
            % Create a new visual representation using patch
            set(self.Visual, 'Vertices', transformedVertices(:, 1:3));
        end
        
        function MarkAsMoved(self)
            self.Moved = true;
        end
        
    
    end

end
