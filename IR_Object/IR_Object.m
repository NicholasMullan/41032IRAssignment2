% Based on the brick class from Assignment 1. Updated to be generalised to
% any object. 


classdef IR_Object < handle

    properties
        
        BasePose %Hold the value of the base pose
        BaseOrientation %Hold the value of the base orientation
        Moved = false % Change to true when move has been completed
        Colour
        Name
        workspace = [-1.8 1.8 -1.8 1.8 -0.3 2];
        model
        Type
    end

    methods 
        %Overload constructor to allow for creating an object type with
        %string input, the name of it, the pose and a colour override in
        %testing. 
        %Note the type MUST include .ply in the name
        
        function self = IR_Object(type, name, pose, colour)
        self.Name = name;
        self.BasePose = pose;
        %self.BaseOrientation = orientation;
        self.Type = type;

         % Load vertices and faces from the PLY file
        [faceData, vertexData] = plyread(self.Type, 'tri');
        link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
        self.model = SerialLink(link1,'name',name);
        self.model.faces = {[], faceData};
        self.model.points = {[], vertexData};
        self.model.base = pose;
        self.model.base = self.model.base.T;

        hold on;

        plot3d(self.model, 0, 'workspace',self.workspace,'view',[-30,30],'delay',0,'noarrow','nowrist');
        end
    
    end

end
