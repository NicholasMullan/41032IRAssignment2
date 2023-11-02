% Connor Keogh      13220482
% Nicholas Mullan   13552361
% Patrick Hore      13623943

%INSTRUCTIONS!!!!!!!!
%To intialise the system, type the below into terminal. 
% run(GUI())

classdef Assignment2Group < handle  
    properties
        %Robots
        r1;
        r2; 
        gripper1;
        gripper2; 
        r1ResetPose;
        r2ResetPose;

        %Table
        BarTableheight = 0.74;
        TallBarTable = 0.95;
        OffsetTable = 0.02;

        %Bottles
        bottleTypes = {
            'CanYellow.ply';
            'CanRed.ply';
            'CanBlue.ply';
            'CanGreen.ply';
            };
        NumberOfBottles = 4; %how many do we want to simulate

        BottleArray;
        EmptyCan; %The empty can / bottle that will be used to fill
        Bottle_; %The bottle that has been picked by the user
        InitialObjectLocationsArray;
        IntermediateObjectLocations1;
        IntermediateObjectLocations2;
        FinalObjectLocationsArray;

        StateMachine; %For higher level tasks to be completed
        SubStateMachine; %For lower level tasks to be completed. Clear at end of function

        EStopPressed; 
        EStopReleased; 
    end
    methods
    
        function self = Assignment2Group()
                %% The main function for this setup
                hold on

                EStopPressed = false;
                EStopReleased = false; 

                %Reset our state machines
                self.SubStateMachine = 0;
                self.StateMachine = 0;

                self.ClearAndClose();
                self.SetupEnvironment();
                self.SetupRobots();
                self.PlaceBottles();

                %gui = GUI()
                %gui.Assignment = this; 

                display("Press to order a drink.");
            end
            
            % General set up of environment
            function ClearAndClose(self)
                %% Reset simulation environment
                clc;
                clf;
                clear all; 
                hold on
                % close all; %Dont close all. Dock the figure for ease of use
                display("ClearAndClose complete");
        
            end
            
            function SetupEnvironment(self)
            %% Setting up enviroment 
            hold on
            
            rs= 2; %Roomsize
            wh= 2; % Wall height
            wb=0; %wall base
            fh=0.01; %floor height
            % Create a textured surface for the floor.
             surf([-rs,-rs;rs,rs] ,[-rs,rs;-rs,rs] ,[fh,fh;fh,fh],'CData',imread('ConcreteFloor.jpg'),'FaceColor','texturemap');
            
            % Define the axis limits for the plot.
            %axis([-rs, rs; -rs, rs; wb, wh]);
            %Setting Up Wall 1 
            % Define wallX, wallY, and wallZ arrays for the wall surface.
            
             
            % Create a textured surface for the wall.
            surf([-rs, -rs; rs, rs],[-rs, -rs; -rs, -rs],[wb, wh; wb, wh],'CData',imread('Brickwall.jpg'),'FaceColor','texturemap');
            
            %Setting Up Wall 2
            % Define wallX, wallY, and wallZ arrays for the wall surface.
            
            surf([-rs, -rs; -rs, -rs],[-rs, -rs; rs, rs],[wb, wh; wb, wh],'CData',imread('Brickwall.jpg'),'FaceColor','texturemap');
            
            % Place Evironment Alterations can be made in blender
            PlaceObject('Environment.ply',[-0.5,1,fh]);
            
            %Placing Human
            PlaceObject('personMaleOld.ply',[1,1,fh]);
            
            
            %Place cctv camera
            cctvXYZ=[-rs+0.1,-rs,wh-0.5];
            %Placing CCTV
            PlaceObject('cctv.ply',cctvXYZ);
            display("SetupEnvironment complete");
        
                    
            end
            
            function SetupRobots(self)
            %% Creates the robots in their positions
            %Robot 1 is the UR5
            hold on
            
           
            
            r1 = UR3();
            r1.model.base = r1.model.base * SE3(-0.5,-0.2,self.BarTableheight + self.OffsetTable);
            self.r1ResetPose = [0,     -pi/2,     0,     0,     0,     0];
            r1.model.animate(self.r1ResetPose);
            
            r1.model.links(1).qlim = deg2rad([-360 360]);
            r1.model.links(2).qlim = deg2rad([-195 0]);
            r1.model.links(3).qlim = deg2rad([-150 150]);
            r1.model.links(4).qlim = deg2rad([-359 359]);
            r1.model.links(5).qlim = deg2rad([-359 359]);
            r1.model.links(6).qlim = deg2rad([-5 5]);

            gripper1 = Gripper("URGripper");
            gripper1.gripperbase_.base = r1.model.fkineUTS(r1.model.getpos) * transl(0,0,-0.05);% * troty(pi);
            gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
            gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
            
            gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
            gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
            gripper1.rightFinger.animate(gripper1.rightFinger.getpos);
             
             
            % Robot 2 is custom robot
            hold on
            r2 = LinearLite6();
            r2.model.base = r2.model.base *  SE3(-1.15,self.BarTableheight + self.OffsetTable,0);
            self.r2ResetPose = [0,     -pi/2,     0,     0,     0,     0,    0];
            r2.model.animate(r2.model.getpos);
            
            for i =1:7
                r2.model.links(i).qlim = deg2rad([-180 180]);
            end
            
            % Set up gripper 2
            gripper2 = Gripper("LinearLiteGripper");
            
            gripper2.gripperbase_.base = r2.model.fkineUTS(r2.model.getpos) * trotz(pi/2) * transl(0,0,0.05);
            gripper2.leftFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0);
            gripper2.rightFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0);
            
            gripper2.gripperbase_.animate(gripper2.gripperbase_.getpos);
            gripper2.leftFinger.animate(gripper2.leftFinger.getpos);
            gripper2.rightFinger.animate(gripper2.rightFinger.getpos);

            self.r1 = r1;
            self.r2 = r2; 
            self.gripper1 = gripper1;
            self.gripper2 = gripper2; 
            
        display("SetupRobots complete");
        
            end
            
            function PlaceBottles(self)
                %% Use this to place each of the bottles in a position
                hold on
        
                StartingX = -1.5;
            
            
                self.InitialObjectLocationsArray = [
                    %Location XYZ   
                    StartingX,    0,     self.TallBarTable;           %Object 1
                    StartingX,    0.4,     self.TallBarTable;          %Object 2
                    StartingX,    0.8,        self.TallBarTable;          %Object 3
                    StartingX,    1.2,      self.TallBarTable          %Object 4                    
                ];
                
                % Where Robot 1 will take the glass to
                self.IntermediateObjectLocations1 = [
                    -1, -0.4,   self.BarTableheight;   
                ];
            
                % Where Robot 1 will take the bottle to
                self.IntermediateObjectLocations2 = [
                    -1.2, -0.4,   self.BarTableheight;   
                ];
            
                %Setting up the code to be able to run 3 different orders for
                %the initial setup
                self.FinalObjectLocationsArray = [
                    0, -0.3,    0.815;       
                ];
          
                for i = 1:self.NumberOfBottles
                    self.BottleArray{i} = IR_Object([self.bottleTypes{i}],['Bottle', num2str(i)], self.InitialObjectLocationsArray(i,:)); 
                end
           
                self.EmptyCan = IR_Object('CanClear.ply','EmptyCan', self.FinalObjectLocationsArray(1,:)); 
            
            
            display("PlaceBottles complete");
        
            end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Callback function on GUI
            function CollectOrderFromCustomer(self, Drink)
                %% 
                hold on;
                Steps = 50;
                %reduce the work by adding a guess
                PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];
    
                FingerRotationOpen = 0.1;
                FingerRotationClosed = -0.1;
                r1 = self.r1;
                r2 = self.r2;
                gripper1 = self.gripper1;
                gripper2 = self.gripper2;
    
                %input from GUI selects which drink should be ordered
    
                % Based on the drink choice, set the initial location in this function

                switch (Drink)
                   
                    case "Solo"
                        BottleNumber = 4;
                    case "CocaCola"
                        BottleNumber = 3;
                     case "Lemonade"
                        BottleNumber = 2;
                    case "Fanta"
                        BottleNumber = 1;
                    otherwise 
                        display("Error: " + Drink + " - out of stock");
                end
    
                %Find the bottle using its name and determine the relevant detai
                self.Bottle_ = self.BottleArray{length(self.BottleArray)-BottleNumber+1};
                
                display("Bottle found: " + Drink + " at position " + BottleNumber);

                %self.StateMachine = 2;

                while (self.StateMachine <= 3)
               if EStopPressed
                % E-stop is pressed, exit the loop or take appropriate action
                break;
            elseif self.EStopReleased
                % E-stop is released
                if ~self.continuePressed
                    % Continue button is pressed, take appropriate action
                    break;
                end
                % Add any other actions to be taken when E-stop is released
            end

                    switch(self.StateMachine)
                        case 0
                            self.Robot1GetGlass();
                        case 1
                            self.Robot2CollectBottle();
                        case 2
                            self.Robot2PourBottle();
                        case 3
                            self.Robot1ReturnCup();

                    end
                    self.StateMachine = self.StateMachine+1;

                end

                self.StateMachine = 0;
                display("Enjoy your drink");

                pause(5);

                self.EmptyCan.model.base = SE3(self.FinalObjectLocationsArray(1,:)); 
                self.EmptyCan.model.animate(self.EmptyCan.model.getpos);

                self.Bottle_.vertexColours = self.Bottle_.InitialVertexColours;
                self.Bottle_.h.link(2).Children.FaceVertexCData = self.Bottle_.vertexColours;
                self.Bottle_.h.link(2).Children.FaceColor = 'interp';
                drawnow();

                self.EmptyCan.vertexColours = self.EmptyCan.InitialVertexColours;
                self.EmptyCan.h.link(2).Children.FaceVertexCData = self.EmptyCan.vertexColours;
                self.EmptyCan.h.link(2).Children.FaceColor = 'interp';
                drawnow();
                display("Bottles replaced and scene reset.");
                display("Press to order a drink.");

            end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % The sub functions for the robot. Called from the state
            % machine in 'CollectOrderFromCustomer'
             function Robot1GetGlass(self)
            %%
            % Use Robot 1 to move glass from customer to intermediate pose
            % Sub steps include 
            % 1. Go to glass
            % 2. Close Gripper
            % 3. Move glass to intermediate
            
            hold on;
            Steps = 50;

            r1 = self.r1;
            gripper1 = self.gripper1;
            EmptyCan = self.EmptyCan;

            while (self.SubStateMachine <= 2)
                 if EStopPressed
                        return;
                end
                    switch (self.SubStateMachine)
                        case 0
                            disp("Step 1.1: Moving robot 1 to cup")
                            
                            %Move toward final position
                            PoseGuess = [deg2rad(20),   deg2rad(-110),   deg2rad(-110),   deg2rad(-145),   deg2rad(-80),   deg2rad(-5)];
                            StepPose = EmptyCan.model.base.T * transl(-0.15,0,0.15) * trotx(pi/2) * troty(pi/2);
                            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
                            jointTrajectory = jtraj(r1.model.getpos, Robot1Final, Steps); %work out the path it takes
                            
                            %Creep to final position
                            StepPose = EmptyCan.model.base.T  * transl(-0.08,0,0.08) * trotx(pi/2) * troty(pi/2);
                            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
                            jointTrajectory2 = jtraj(jointTrajectory(Steps,:), Robot1Final, Steps/2); %Creep trajectorys
            
                            jointTrajectory = [jointTrajectory; jointTrajectory2 ];

                            MoveRobot(r1, gripper1, false, jointTrajectory, NaN);
                        case 1
                            % Close gripper
                            disp("Step 1.2: Robot arrived at cup, closing gripper")
                            AlterGripper(gripper1, true); %false to open gripper, true to close
                        
                        case 2
                            disp("Step 1.3: Robot carrying cup to pouring position")           
                            
                            %Lift the cup from its initial pose
                            PoseGuess = [deg2rad(20),   deg2rad(-110),   deg2rad(-110),   deg2rad(-145),   deg2rad(-80),   deg2rad(-5)];
                            StepPose = EmptyCan.model.base.T * transl(-0.15,0,0.15) * trotx(pi/2) * troty(pi/2);
                            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
                            jointTrajectory = jtraj(r1.model.getpos, Robot1Final, Steps/2); %work out the path it takes

                            %Move to intermediate pose
                            PoseGuess = [-2.5993,   -2.2692,   -1.4595, -2.5544,   -1.5340,   -0.0000];
                            Bottle_Pose = SE3(self.IntermediateObjectLocations1).T * trotx(pi/2) * transl(0,0.15,0) * troty(-pi/2) ;
               
                            Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
                            jointTrajectory2 = jtraj(jointTrajectory(Steps/2,:), Robot1Intermediate, Steps); %work out the path it takes
            
                            %Creep to intermediate pose
                            % PoseGuess = [-2.5993,   -2.2692,   -1.4595, -2.5544,   -1.5340,   -0.0000];
                            % Bottle_Pose = SE3(self.IntermediateObjectLocations1).T * trotx(pi/2) * transl(0,0.08,0) * troty(-pi/2) ;
                            % Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
                            % jointTrajectory3 = jtraj(jointTrajectory2(Steps,:), Robot1Intermediate, Steps/2); %work out the path it takes
                            % 
                            jointTrajectory = [jointTrajectory; jointTrajectory2]; %; jointTrajectory3];
                            MoveRobot(r1, gripper1, true, jointTrajectory, EmptyCan);        
                    end
                    self.SubStateMachine = self.SubStateMachine+1;
                end
                self.SubStateMachine = 0;


            end

            function Robot2CollectBottle(self)
                %%
                % Use Robot 2 to move bottle from initial pose to intermediate pose
                % Sub steps include 
                % 1. Go to bottle
                % 2. Close Gripper
                % 3. Move bottle to intermediate
                hold on;
                Steps = 50;

                r2 = self.r2;
                gripper2 = self.gripper2;
                Bottle_ = self.Bottle_;
                
                %3 sub missions in this function
                while (self.SubStateMachine < 3)
                    if EStopPressed
                        return;
                    end
                    switch (self.SubStateMachine)
                        case 0
                            disp("Step 2.1: Moving robot 2 to bottle")
    
                            % Use Robot 2 to move bottle from initial pose to intermediate pose
                            % Robot close to position
                            PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];
                            Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.1,0,-.24);
                            robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
                            jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps); %work out the path it takes
                
                            % creep speed to get in close
                            PoseGuess = [ -0.4049    0.8781   -0.6030   -1.4791   -1.1850    0.9801    0.6300];
                            Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.1,0,-0.18);
                            robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
                            creepJointTrajectory = jtraj(jointTrajectory(50,:), robot2Bottle, Steps/2); %work out the path it takes
                
                            jointTrajectory = [jointTrajectory; creepJointTrajectory ];
            
                            MoveRobot(r2, gripper2, false, jointTrajectory, NaN);
                        case 1
                            % Close gripper
                            disp("Step 2.2: Robot arrived at bottle, closing gripper")
                            AlterGripper(gripper2, true); %false to open gripper, true to close
                        case 2
                            disp("Step 2.3: Bottle picked up, moving to intermediate position")
                            %creep Movement up after picking bottle
                            Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.2,0,-.3);
                            robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
                            jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps/2); %work out the path it takes
                
                             %Movement to near intermediate position
                            PoseGuess = [0.0420    1.3929   -1.4104   -0.9893    0.2391    0.8434    1.7315];
                            StepPose = SE3(self.IntermediateObjectLocations2).T * troty(-pi/2) * trotx(pi/2) * transl(0.24,0,-0.25);
                            Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
                            jointTrajectory2 = jtraj(jointTrajectory(25,:), Robot2Intermediate, Steps); %work out the path it takes
                
                            jointTrajectory = [jointTrajectory; jointTrajectory2]; % jointTrajectory3];
                            jtrajSize = size(jointTrajectory);
                
                            MoveRobot(r2, gripper2, true, jointTrajectory,Bottle_);
                    end
                    self.SubStateMachine = self.SubStateMachine+1;

                end
                self.SubStateMachine = 0;
               
            end
           
            function Robot2PourBottle(self)
            % Use Robot 2 to pour from one glass to another
            % Sub steps include 
            % 1. Raise full can to pose
            % 2. Tip full and reverse tip empty. Slowly rotate back to upright pose
            % 3. Move cup back to customer
            % 4. Place bottle back to its base pose
            % 5. Move robot to back to base pose
            
            hold on;
            Steps = 50;

            r2 = self.r2;
            gripper2 = self.gripper2;
            Bottle_ = self.Bottle_;

            r1 = self.r1;
            gripper1 = self.gripper1;
            EmptyCan = self.EmptyCan;

            ReturnPos = r2.model.getpos;

            while (self.SubStateMachine <= 4)
                if EStopPressed
                    return;
                end
                switch (self.SubStateMachine)
                    case 0
                        disp("Step 3.1: Lifting full can")  
                        %creep Movement up after picking bottle
                        PoseGuess = [-0.1800,    1.5826,   -1.2519,   -0.4016,   -0.0236,    0.0826,         0];
                        jointTrajectory = jtraj(r2.model.getpos, PoseGuess, Steps); %work out the path it takes

                        PoseGuess = [-0.1800,    1.6299,   -1.2519,   -0.4016,   -0.0236,    0.0826,         -0.7322];
                        jointTrajectory2 = jtraj(jointTrajectory(Steps,:), PoseGuess, Steps/2); %work out the path it takes
                            
                        jointTrajectory = [jointTrajectory; jointTrajectory2];

                        MoveRobotExchangeLiquid(r2, gripper2, true, jointTrajectory, Bottle_, EmptyCan);
                    case 1
                        % 
                        disp("Step 3.2: Fix pose of Robot 2")
                        
                        jointTrajectory = jtraj(r2.model.getpos, ReturnPos, Steps); %work out the path it takes
                        MoveRobot(r2, gripper2, true, jointTrajectory, Bottle_);

                    case 2
                        disp("Step 3.3:  Move bottle back to position")

                        % Use Robot 2 to move bottle from current pose to initial pose

                        PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];
                        Bottle_Pose = SE3(Bottle_.homeQ).T * troty(-pi/2) * transl(0.2,0,-.24);;
                        robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
                        jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps); %work out the path it takes

                        % creep speed to get in close
                        PoseGuess = [ -0.4049    0.8781   -0.6030   -1.4791   -1.1850    0.9801    0.6300];
                        Bottle_Pose = SE3(Bottle_.homeQ).T * troty(-pi/2) * transl(0.1,0,-0.18);
                        robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
                        creepJointTrajectory = jtraj(jointTrajectory(50,:), robot2Bottle, Steps/2); %work out the path it takes

                        jointTrajectory = [jointTrajectory; creepJointTrajectory ];
                        MoveRobot(r2, gripper2, true, jointTrajectory, Bottle_);
                    case 3
                        disp("Step 3.4:  Opening gripper to release bottle")
                        AlterGripper(gripper2, false); %false to open gripper, true to close
                    case 4
                        disp("Step 3.5: Return robot 2 to base pose")

                        %Creep away from bottle
                        PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];
                        Bottle_Pose = SE3(Bottle_.homeQ).T * troty(-pi/2) * transl(0.2,0,-.24);;
                        robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
                        jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps/2); %work out the path it takes

                        %Return to base
                        jointTrajectory2 = jtraj(jointTrajectory(Steps/2,:), self.r2ResetPose, Steps); %work out the path it takes
                        
                        jointTrajectory = [jointTrajectory; jointTrajectory2];
                        MoveRobot(r2, gripper2, false, jointTrajectory, NaN);
                end
                self.SubStateMachine = self.SubStateMachine+1;
            end

            self.SubStateMachine = 0;
            end

            function Robot1ReturnCup(self)
            % Use Robot 1 to move glass from intermediate to customer pose
            % Sub steps include 
            % 1. Move to customer pose
            % 2. Open Gripper
            % 3. Move glass to intermediate
            
            hold on;
            Steps = 50;

            r1 = self.r1;
            gripper1 = self.gripper1;
            EmptyCan = self.EmptyCan;

            while (self.SubStateMachine <= 2)
                    if EStopPressed
                        return;
                    end
                    switch (self.SubStateMachine)
                        case 0
                            disp("Step 4.1: Moving robot 1 to cup")
                            
                            %Move toward final position
                            PoseGuess = [deg2rad(20),   deg2rad(-110),   deg2rad(-110),   deg2rad(-145),   deg2rad(-80),   deg2rad(-5)];
                            StepPose = SE3(EmptyCan.homeQ).T * transl(-0.15,0,0.15) * trotx(pi/2) * troty(pi/2);
                            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
                            jointTrajectory = jtraj(r1.model.getpos, Robot1Final, Steps); %work out the path it takes
                            
                            %Creep to final position
                            StepPose = SE3(EmptyCan.homeQ).T  * transl(-0.08,0,0.08) * trotx(pi/2) * troty(pi/2);
                            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
                            jointTrajectory2 = jtraj(jointTrajectory(Steps,:), Robot1Final, Steps/2); %Creep trajectorys
            
                            jointTrajectory = [jointTrajectory; jointTrajectory2 ];

                            MoveRobot(r1, gripper1, true, jointTrajectory, EmptyCan);
                        case 1
                            % Open gripper
                            disp("Step 4.2:  Opening gripper to release bottle")
                            AlterGripper(gripper1, false); %false to open gripper, true to close
                        
                        case 2
                            disp("Step 4.3: Return robot 1 to base pose")

                            %Creep away from cup
                            PoseGuess = [deg2rad(20),   deg2rad(-110),   deg2rad(-110),   deg2rad(-145),   deg2rad(-80),   deg2rad(-5)];
                            StepPose = SE3(EmptyCan.homeQ).T * transl(-0.15,0,0.15) * trotx(pi/2) * troty(pi/2);
                            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
                            jointTrajectory = jtraj(r1.model.getpos, Robot1Final, Steps/2); %work out the path it takes
                            
                            %Return to base pose
                            jointTrajectory2 = jtraj(jointTrajectory(Steps/2,:), self.r1ResetPose, Steps); %work out the path it takes

                            jointTrajectory = [jointTrajectory; jointTrajectory2];
                            MoveRobot(r1, gripper1, false, jointTrajectory, NaN);        
                    end
                    self.SubStateMachine = self.SubStateMachine+1;
                end
                self.SubStateMachine = 0;
            end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Helper Functions which do not need to be in the class
function AlterGripper(gripper, gripperClosed)
%gripperClosed = true when you want to close the gripper. False when you
%want to open
    RotationY = 0.12;
    if gripperClosed
        RotationY = -0.12;
    end

    steps_ = 50; 

    for i = 1:steps_ 
                gripper.rightFinger.base = gripper.rightFinger.base.T * troty(RotationY/steps_);
                gripper.leftFinger.base = gripper.leftFinger.base.T * troty(-RotationY/steps_);
                gripper.rightFinger.animate(gripper.leftFinger.getpos);
                gripper.leftFinger.animate(gripper.rightFinger.getpos);

                drawnow();
    end
end

function MoveRobot(robot, gripper, gripperClosed, jTraj, bottle)       
        
        FingerRotation = -0.1;
        if gripperClosed
            FingerRotation = 0.1;
        end
            

        GripperOffset =  transl(0,0,-0.05);
        BottleOffset =  trotx(-pi/2)  * transl(0,-0.08,-0.08);

        if robot.plyFileNameStem(1) == 'L'
            GripperOffset =  trotz(pi/2) * transl(0,0,0.05);
            BottleOffset =   troty(pi/2)  * transl(-0.18,0,-0.1);
        end

        bottlePassed = isa(bottle, 'IR_Object');
        
        jtrajSize = size(jTraj);
        for i = 1:jtrajSize
                if EStopPressed
                    return;
                end

                %adjust and animate the postion of the gripper
                gripper.gripperbase_.base = robot.model.fkineUTS(jTraj(i,:)) * GripperOffset;
                gripper.leftFinger.base = gripper.gripperbase_.base.T * transl(0,0.1,0) * troty(FingerRotation);
                gripper.rightFinger.base = gripper.gripperbase_.base.T * transl(0,0.1,0) * troty(-FingerRotation);

                gripper.gripperbase_.animate(gripper.gripperbase_.getpos);
                gripper.leftFinger.animate(gripper.leftFinger.getpos);
                gripper.rightFinger.animate(gripper.rightFinger.getpos);

                if bottlePassed
                    bottle.model.base = robot.model.fkineUTS(jTraj(i,:)) * BottleOffset;
                    animate(bottle.model,jTraj(i,:));
                end
                
                %animate the arm
                animate(robot.model,jTraj(i,:));
                
                drawnow();
        
        end
end

function MoveRobotExchangeLiquid(robot, gripper, gripperClosed, jTraj, FullBottle, EmptyBottle)       
        
        FingerRotation = -0.1;
        if gripperClosed
            FingerRotation = 0.1;
        end
            

        GripperOffset =  transl(0,0,-0.05);
        BottleOffset =  trotx(-pi/2)  * transl(0,-0.08,-0.08);

        if robot.plyFileNameStem(1) == 'L'
            GripperOffset =  trotz(pi/2) * transl(0,0,0.05);
            BottleOffset =   troty(pi/2)  * transl(-0.18,0,-0.1);
        end

        bottlePassed = isa(FullBottle, 'IR_Object');

        % Get the typical colour of the full bottle
        Avg1 = mean(FullBottle.vertexColours(:,1));
        Avg2 = mean(FullBottle.vertexColours(:,1));
        Avg3 = mean(FullBottle.vertexColours(:,3));
        colour = [Avg1, Avg2, Avg3]; 

        VertexArraySize_Full = size(FullBottle.vertexColours);
        VertexArraySize_Empty = size(EmptyBottle.vertexColours);

        oldFluidHeight = 1;

        jtrajSize = size(jTraj);
        for i = 1:jtrajSize
                if EStopPressed
                    return;
                end

                %adjust and animate the postion of the gripper
                gripper.gripperbase_.base = robot.model.fkineUTS(jTraj(i,:)) * GripperOffset;
                gripper.leftFinger.base = gripper.gripperbase_.base.T * transl(0,0.1,0) * troty(FingerRotation);
                gripper.rightFinger.base = gripper.gripperbase_.base.T * transl(0,0.1,0) * troty(-FingerRotation);

                gripper.gripperbase_.animate(gripper.gripperbase_.getpos);
                gripper.leftFinger.animate(gripper.leftFinger.getpos);
                gripper.rightFinger.animate(gripper.rightFinger.getpos);


                % Chan ge the fluid colour
                fluidHeight = round(((i / jtrajSize(1)) * VertexArraySize_Full(1)),0);
                for vertexIdx = oldFluidHeight:fluidHeight
                    HoldColour = FullBottle.vertexColours(vertexIdx,:);
                    FullBottle.vertexColours(vertexIdx,:) = [1,1,1];
                    EmptyBottle.vertexColours(vertexIdx, :) = HoldColour;
                    % Set the color to red for this vertex
                end
                oldFluidHeight = fluidHeight;
                FullBottle.h.link(2).Children.FaceVertexCData = FullBottle.vertexColours;
                FullBottle.h.link(2).Children.FaceColor = 'interp';
            
                EmptyBottle.h.link(2).Children.FaceVertexCData = EmptyBottle.vertexColours;
                EmptyBottle.h.link(2).Children.FaceColor = 'interp';

                if bottlePassed
                    FullBottle.model.base = robot.model.fkineUTS(jTraj(i,:)) * BottleOffset;
                    animate(FullBottle.model,jTraj(i,:));
                end
               
                %animate the arm
                animate(robot.model,jTraj(i,:));
                drawnow();
        end

        for i = 1:VertexArraySize_Full
            FullBottle.vertexColours(i,:) = [1,1,1];

        end
        for i = 1:VertexArraySize_Empty
            EmptyBottle.vertexColours(i,:) = colour;
        end
        drawnow();
end
 