%Connor Keogh 13220482
%Built based upon RobotCow

classdef Gripper < handle
    properties
    gripperbase_;
    leftFinger;
    rightFinger;

    workspace = [-2 2 -2 2 -0.3 2];
    end


    methods
        function self = Gripper()
            L1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.gripperbase_ = SerialLink(L1, 'name', 'gripbase');

            finger1Link = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.leftFinger = SerialLink(finger1Link, 'name', 'left');
            self.leftFinger.base = self.gripperbase_.base * SE3(0,0.1,0);
            % self.leftFinger.base = self.leftFinger.base.T * troty(0.1);
            
            finger2Link = Link('alpha',0,'a',0,'d',0,'offset',0);
            self.rightFinger = SerialLink(finger2Link, 'name', 'right');  
            self.rightFinger.base = self.gripperbase_.base * SE3(0,0.1,0);
            % self.rightFinger.base = self.rightFinger.base.T * troty(-0.1);
            
            self.PlotAndColourRobot();
        end

        function PlotAndColourRobot(self)
            a = self.gripperbase_.n; %Use a to overwrite the indexing value
            [ faceData, vertexData, plyData{a+1} ] = plyread('GBase.ply','tri'); 
            self.gripperbase_.faces{a+1} = faceData;
            self.gripperbase_.points{a+1} = vertexData;

            a = self.leftFinger.n; %Use a to overwrite the indexing value
            [ faceData, vertexData, plyData{a+1} ] = plyread('GLeft.ply','tri'); 
            self.leftFinger.faces{a+1} = faceData;
            self.leftFinger.points{a+1} = vertexData;

            a = self.rightFinger.n; %Use a to overwrite the indexing value
            [ faceData, vertexData, plyData{a+1} ] = plyread('GRight.ply','tri');
            self.rightFinger.faces{a+1} = faceData;
            self.rightFinger.points{a+1} = vertexData;

            self.displaygripper();
           
        end

        function displaygripper(self)
            self.gripperbase_.plot3d(zeros(1,self.gripperbase_.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.gripperbase_.delay = 0;

            self.leftFinger.plot3d(zeros(1,self.leftFinger.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.leftFinger.delay = 0;

            self.rightFinger.plot3d(zeros(1,self.rightFinger.n),'noarrow','workspace',self.workspace);
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end
            self.rightFinger.delay = 0;

        end
        
    
    
    
    end




end
