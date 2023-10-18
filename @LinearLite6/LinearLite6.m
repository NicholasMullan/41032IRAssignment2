classdef LinearLite6 < RobotBaseClass
    %% Linear Robot 2  
    properties(Access = public)              
        plyFileNameStem = 'LinearLite6';
    end

    methods
%% Define robot Function 
        function self = LinearLite6(baseTr)
			self.CreateModel();
            if nargin < 1			
				baseTr = eye(4);				
            end
            self.model.base = self.model.base.T * baseTr * trotx(pi/2) * troty(pi/2);
            
            self.PlotAndColourRobot();         
        end

%% Create the robot model
        function CreateModel(self)   
            % Create the Robot2 model mounted on a linear rail
            link(1) = Link([pi 0 0 pi/2 1]); % PRISMATIC Link
            link(2) = Link([0 0.3165 0 0 0]);
            link(3) = Link([0 0.2 0 0 0]);
            link(4) = Link([0 0 0 pi/2 0]);
            link(5) = Link([0 0 -0.1 0 0]);
            link(6) = Link([0 0 0 -pi/2 0]);
            link(7) = Link([0 -0.339 0.013 0 0]);
            
            % Incorporate joint limits
            link(1).qlim = [-0.8 -0.01];
            link(2).qlim = [-360 360]*pi/180;
            link(3).qlim = [-150 150]*pi/180;
            link(4).qlim = [-3.5 300]*pi/180;
            link(5).qlim = [-360 360]*pi/180;
            link(6).qlim = [-124 124]*pi/180;
            link(7).qlim = [-360 360]*pi/180;

            
        
                      
            self.model = SerialLink(link,'name',self.name);
        end
     
    end
end