% Based on the brick class from Assignment 1. Updated to be generalised to
% any object. 
classdef IR_Object < RobotBaseClass

    properties(Access = public)
        plyFileNameStem = '';
        plyData
        BasePose %Hold the value of the base pose
        Name
        Type

        h;
        vertexColours;
        InitialVertexColours; 
    end

    methods 
        function self = IR_Object(type, name, pose)
        hold on;
        self.name = name;
        self.model.name = name;
        self.BasePose = pose;
        self.homeQ = pose;
        self.Type = type;

        self.CreateModel();
        self.model.base = pose;

        self.model.base = self.model.base.T;
        

        self.PlotAndColourRobot();

        self.InitialVertexColours = self.vertexColours;

        %  % Load vertices and faces from the PLY file
        % plyData = plyread(self.Type, 'tri')
        % link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
        % 
        % self.model = SerialLink(link1,'name',name);
        % self.model.faces = {[], faceData};
        % self.model.points = {[], vertexData};
        % 
        % self.model.base = pose;
        % self.model.base = self.model.base.T;
        % 
        % 
        
        % 
        % %plot3d(self.model, 0, 'workspace',self.workspace,'view',[-30,30],'delay',0,'noarrow','nowrist');
        
        end

        %% Create the robot model
        function CreateModel(self)   
            % Create the Robot2 model mounted on a linear rail
            link(1) = Link('alpha',0,'a',0,'d',0,'offset',0);
            
            self.model = SerialLink(link,'name',self.name);
        end


        function PlotAndColourRobot(self)
            if isempty(self.homeQ)
                self.homeQ = zeros(1,self.model.n);
            end

            if exist([self.plyFileNameStem,'.mat'],'file') == 2 && exist([self.plyFileNameStem,self.Type],'file') ~= 2
                warning('There are no ply files for the links but there is a mat file. You should use PlotAndColourRobotMat to create and colour a 3D robot model plot. I am doing this for you now.')
                self.PlotAndColourRobotMat()
                return;
            end

            for linkIndex = 0:self.model.n
                if self.useTool && linkIndex == self.model.n
                    if ~isempty(self.toolFilename)
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.toolFilename],'tri'); %#ok<AGROW>
                    else
                        [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,self.Type],'tri'); %#ok<AGROW>
                    end
                else
                    [ faceData, vertexData, plyData{linkIndex+1} ] = plyread([self.plyFileNameStem,self.Type],'tri'); %#ok<AGROW>
                end

                % Obtain faceData and vertexData for the current link and save to the cell.
                self.model.faces{linkIndex+1} = faceData;
                self.model.points{linkIndex+1} = vertexData;
            end

            self.h = self.InitiliseRobotPlot();
            if 1 < length(self.h)
                self.MultirobotWarningMessage();
                self.h = self.h{1};
            end

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                self.vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData
                try 
                     self.vertexColours = [plyData{linkIndex+1}.vertex.red ...
                                     , plyData{linkIndex+1}.vertex.green ...
                                     , plyData{linkIndex+1}.vertex.blue]/255;

                catch ME_1
                    disp(ME_1);
                    disp('No vertex colours in plyData');
                    try 
                         self.vertexColours = [plyData{linkIndex+1}.face.red ...
                                     , plyData{linkIndex+1}.face.green ...
                                     , plyData{linkIndex+1}.face.blue]/255;
                    catch ME_1
                        disp(ME_1);
                        disp(['Also, no face colours in plyData, so using a default colour: ',num2str(vertexColours)]);
                    end
                end

                self.h.link(linkIndex+1).Children.FaceVertexCData = self.vertexColours;
                self.h.link(linkIndex+1).Children.FaceColor = 'interp';
            end
            drawnow();
        end
    
    end

end
