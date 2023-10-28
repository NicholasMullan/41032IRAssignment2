% Connor Keogh      13220482
% Nicholas Mullan   13552361
% Patrick Hore      13623943

classdef Assignment2Group

    properties
        %% Set the initial position for each Object

        % Define the height of the bar / table here for use of robot placement
        BarTableheight = 0.74; %%(800mm) 
        TallBarTable = 0.95;
        OffsetTable = 0.02;


        bottleTypes = {
                'CanYellow.ply';
                'CanRed.ply';
                'CanBlue.ply';
                'CanGreen.ply';
                };
        arraySize = size(bottleTypes());
        NumberOfBottles = arraySize(1); %how many do we want to simulate


    end
    
    

        methods
        function main(self)
            %% The main function for this setup
            self.ClearAndClose(self);
            self.SetupEnvironment();
            self.SetupRobots();
            self.SetArrayValues();
            self.PlaceBottles();
            % for i=0 : NumberOfBottles 
            %     CollectOrderFromCustomer(i);
            % end
        
        end
        
        function ClearAndClose(self)
            %% Reset simulation environment
            clc;
            clf;
            clear all; 
            hold on
            % close all; %Dont close all. Dock the figure for ease of use
        end
        
        %% Set up Environment
        
        % initial object locations. 
        
        
        function SetupEnvironment()
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
        
                
        end

        function SetupRobots()
        %% Creates the robots in their positions
        %Robot 1 is the UR5
        hold on

        r1 = UR3();
        r1.model.base = r1.model.base * SE3(-0.5,-0.3,BarTableheight + OffsetTable);
        r1ResetPose = [0,     -pi/2,     0,     0,     0,     0];
        r1.model.animate(r1ResetPose);

        r1.model.links(1).qlim = deg2rad([-360 360]);
        r1.model.links(2).qlim = deg2rad([-195 0]);
        r1.model.links(3).qlim = deg2rad([-150 150]);
        r1.model.links(4).qlim = deg2rad([-360 360]);
        r1.model.links(5).qlim = deg2rad([-360 360]);
        r1.model.links(6).qlim = deg2rad([-5 5]);

        gripper1 = Gripper("URGripper");
        gripper1.gripperbase_.base = r1.model.fkineUTS(r1.model.getpos) * transl(0,0,-0.05);% * troty(pi);
        gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
        gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);

        %gripper1.gripperbase_.base = r1.model.base.T * transl(0,0.70,-0.19) * troty(pi);
        % gripper1.leftFinger.base = r1.model.base.T  * (gripper1.leftFinger.base.T * transl(0,0.70,-0.19)) * troty(pi);
        % gripper1.rightFinger.base = r1.model.base.T * (gripper1.rightFinger.base.T * transl(0,0.70,-0.19)) * troty(pi);
        
        gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
        gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
        gripper1.rightFinger.animate(gripper1.rightFinger.getpos);
         
         
        % Robot 2 is custom robot
        hold on
        r2 = LinearLite6();
        %r2.model.base = r2.model.base * SE3(0,BarTableheight + OffsetTable,1.5);
        r2.model.base = r2.model.base *  SE3(-1.15,BarTableheight + OffsetTable,0);
       
        r2ResetPose = [0,     -pi/2,     0,     0,     0,     0,    0];
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

        end
        
        function SetArrayValues()         
            %% Set up bottle initial locations
            % note no orientation considering bottle is mirrored about Z-axis.             

            StartingX = -1.5;

            InitialObjectLocationsArray = [
                %Location XYZ   
                StartingX,    -0.4,    TallBarTable;           %Object 1
                StartingX,    -0.2,   TallBarTable;          %Object 2
                StartingX,    0,   TallBarTable;          %Object 3
                StartingX,    0.2,    TallBarTable;          %Object 4
                StartingX,    0.4,   TallBarTable;         %Object 5
                StartingX,    0.6,   TallBarTable;         %Object 6
                StartingX,    0.8,    TallBarTable;          %Object 7
                StartingX,    1,   TallBarTable;         %Object 8
                StartingX,   1.2,   TallBarTable;         %Object 9
            ];
            
            % This may change to a single number
            IntermediateObjectLocations = [
                -1.1, -0.4,   BarTableheight;   
            ];
           

            %Setting up the code to be able to run 3 different orders for
            %the initial setup
            FinalObjectLocationsArray = [
                %Location XYZ   Rotation
                -0.1, -0.3,    0.85;       
            ];
        end

        function PlaceBottles(self)
            %% Use this to place each of the bottles in a position
            for i = 1:NumberOfBottles
                BottleArray{i} = IR_Object([bottleTypes{i}],['Bottle', num2str(i)], InitialObjectLocationsArray(i,:)); 
            end

        end
    
        %% GUI Opened
        gui = GUI();

%         %% Move objects
%         function CollectOrderFromCustomer(Num)
%             %% 
%             Steps = 50;
%             %reduce the work by adding a guess
%             PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];
% 
%             %input from GUI selects which drink should be ordered
%             Drink = "Fanta";
%             % BottleNumber = 2;
% 
%             % Based on the drink choice, set the initial location in this
%             % function
%             switch (Drink)
%                 case "Lemonade"
%                     BottleNumber = 1;
%                 case "Solo"
%                     BottleNumber = 2;
%                 case "Fanta"
%                     BottleNumber = 3;
%                 case "CocaCola"
%                     BottleNumber = 4;
%                 otherwise 
%                     display("Error: " + Drink + " - out of stock");
%             end
%                         %BottleNumber = 1;
% 
% 
%             %Find the bottle using its name and determine the relevant detai
%             Bottle_ = BottleArray{length(BottleArray)-BottleNumber+1};
% 
% 
%             display("Bottle found: " + Drink + " at position " + BottleNumber);
% 
%             %Now that we have the bottle, we can determine all the other
%             %information through this
% 
%             % Use Robot 2 to move bottle from initial pose to intermediate pose
%             % Robot close to position
%             Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.1,0,-.24);
%             robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
%             jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps); %work out the path it takes
% 
%             % creep speed to get in close
%             PoseGuess = [ -0.4049    0.8781   -0.6030   -1.4791   -1.1850    0.9801    0.6300];
%             Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.1,0,-0.18);
%             robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
%             creepJointTrajectory = jtraj(jointTrajectory(50,:), robot2Bottle, Steps/2); %work out the path it takes
% 
%             jointTrajectory = [jointTrajectory; creepJointTrajectory ];
%             jtrajSize = size(jointTrajectory);
% 
%             for i = 1:jtrajSize(1)
%                 %adjust and animate the postion of the gripper
% 
%                 gripper2.gripperbase_.base = r2.model.fkineUTS(jointTrajectory(i,:)) * trotz(pi/2) * transl(0,0,0.05);
%                 gripper2.leftFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0);
%                 gripper2.rightFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0);
% 
%                 gripper2.gripperbase_.animate(gripper2.gripperbase_.getpos);
%                 gripper2.leftFinger.animate(gripper2.leftFinger.getpos);
%                 gripper2.rightFinger.animate(gripper2.rightFinger.getpos);
% 
%                 %animate the arm
%                 animate(r2.model,jointTrajectory(i,:));
% 
%                 drawnow();
% 
%             end
% 
% disp("Robot arrived at bottle, closing gripper")
% 
%             % Close gripper
%             for i = 1:Steps 
%                 gripper2.rightFinger.base = gripper2.rightFinger.base.T * troty(-0.1/Steps);
%                 gripper2.leftFinger.base = gripper2.leftFinger.base.T * troty(0.1/Steps);
%                 gripper2.rightFinger.animate(gripper2.leftFinger.getpos);
%                 gripper2.leftFinger.animate(gripper2.rightFinger.getpos);
% 
%                 drawnow();
% 
%             end
% disp("Bottle picked up, moving to intermediate position")
% 
%             %creep Movement up after picking bottle
%             Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.2,0,-.3);
%             robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
%             jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps/2); %work out the path it takes
% 
%              %Movement to near intermediate position
%             PoseGuess = [0.0420    1.3929   -1.4104   -0.9893    0.2391    0.8434    1.7315];
%             StepPose = SE3(IntermediateObjectLocations).T * troty(-pi/2) * trotx(pi/2) * transl(0.14,0,-0.25);
%             Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
%             jointTrajectory2 = jtraj(jointTrajectory(25,:), Robot2Intermediate, Steps); %work out the path it takes
% 
%             %creep speed placement at intermediate position
%             StepPose = SE3(IntermediateObjectLocations).T * troty(-pi/2) * trotx(pi/2) * transl(0.12,0,-0.18);
%             Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
%             jointTrajectory3 = jtraj(jointTrajectory2(50,:), Robot2Intermediate, Steps/2); %work out the path it takes
% 
%             jointTrajectory = [jointTrajectory; jointTrajectory2; jointTrajectory3];
%             jtrajSize = size(jointTrajectory);
% 
%             for i = 1:jtrajSize(1)
%                 %adjust and animate the postion of the gripper
%                 gripper2.gripperbase_.base = r2.model.fkineUTS(jointTrajectory(i,:)) * trotz(pi/2) * transl(0,0,0.05);
%                 gripper2.leftFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0) * troty(0.1);
%                 gripper2.rightFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0) * troty(-0.1);
% 
%                 gripper2.gripperbase_.animate(gripper2.gripperbase_.getpos);
%                 gripper2.leftFinger.animate(gripper2.leftFinger.getpos);
%                 gripper2.rightFinger.animate(gripper2.rightFinger.getpos);
% 
%                 %animate the arm
%                 animate(r2.model,jointTrajectory(i,:));
% 
%                 Bottle_.model.base = r2.model.fkineUTS(jointTrajectory(i,:)) * troty(pi/2)  * transl(-0.18,0,-0.1)
%                 animate(Bottle_.model,jointTrajectory(i,:));
% 
%                 drawnow();
% 
%             end
% 
% disp("Bottle at intermediate position, opening grippper")
% 
% 
%             %  Open gripper
%             for i = 1:Steps 
%                 gripper2.rightFinger.base = gripper2.rightFinger.base.T * troty(0.1/Steps);
%                 gripper2.leftFinger.base = gripper2.leftFinger.base.T * troty(-0.1/Steps);
%                 gripper2.rightFinger.animate(gripper2.leftFinger.getpos);
%                 gripper2.leftFinger.animate(gripper2.rightFinger.getpos);
% 
%                 drawnow();
% 
%             end
% 
% disp("Moving robot out of the road")
%             % creep away from bottle
%             PoseGuess = [0.0420    1.3929   -1.4104   -0.9893    0.2391    0.8434    1.7315];
%             StepPose = SE3(IntermediateObjectLocations).T * troty(-pi/2) * trotx(pi/2) * transl(0.14,0,-0.25);
%             Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
%             jointTrajectory = jtraj(r2.model.getpos, Robot2Intermediate, Steps/2); %work out the path it takes
% 
%             % Move to reset pose
%             StepPose = SE3(-1.15,0.09,1.502).T * troty(-pi/2) * trotx(pi/2) * transl(0.14,0,-0.25);
%             Robot2Reset = r2.model.ikcon(StepPose,PoseGuess);
%             jointTrajectory2 = jtraj(jointTrajectory(25,:), Robot2Reset, Steps); %work out the path it takes
% 
%             jointTrajectory = [jointTrajectory; jointTrajectory2];
%             jtrajSize = size(jointTrajectory);
% 
%             %Move robot 2 out of the road
%             for i = 1:jtrajSize
%                 %adjust and animate the postion of the gripper
%                 gripper2.gripperbase_.base = r2.model.fkineUTS(jointTrajectory(i,:)) * trotz(pi/2) * transl(0,0,0.05);
%                 gripper2.leftFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0);
%                 gripper2.rightFinger.base = gripper2.gripperbase_.base.T * transl(0,0.1,0);
% 
%                 gripper2.gripperbase_.animate(gripper2.gripperbase_.getpos);
%                 gripper2.leftFinger.animate(gripper2.leftFinger.getpos);
%                 gripper2.rightFinger.animate(gripper2.rightFinger.getpos);
% 
%                 %animate the arm
%                 animate(r2.model,jointTrajectory(i,:));
% 
%                 drawnow();
% 
%             end
%             %holdPos = Bottle_.model.base;
% 
% %Bottle_.model.base = holdPos;
% %animate(Bottle_.model,Bottle_.model.getpos);
% 
% disp("Robot 1 moving into position")
%             % Use Robot 1 to move bottle from intermediate pose to final pose
%             PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
%             Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.15,-0.11);
%             Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
%             jointTrajectory = jtraj(r1.model.getpos, Robot1Intermediate, Steps); %work out the path it takes
% 
%             %Creep to bottle position
%             PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
%             Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.1,-0.1);
%             Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
%             jointTrajectory2 = jtraj(jointTrajectory(50,:), Robot1Intermediate, Steps/2); %work out the path it takes
% 
%             jointTrajectory = [jointTrajectory; jointTrajectory2];
%             jtrajSize = size(jointTrajectory);
% 
%             %Move robot A out of the road
%             for i = 1:jtrajSize
%                 %adjust and animate the postion of the gripper
%                 gripper1.gripperbase_.base = r1.model.fkineUTS(jointTrajectory(i,:)) * transl(0,0,-0.05);
%                 gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
%                 gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
% 
%                 gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
%                 gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
%                 gripper1.rightFinger.animate(gripper1.rightFinger.getpos);
% 
%                 %animate the arm
%                 animate(r1.model,jointTrajectory(i,:));
% 
%                 drawnow();
% 
%             end
% disp("Robot 1 in position. Gripper closing")
% r1.model.getpos
%             % Close gripper
%             for i = 1:Steps 
%                 gripper1.leftFinger.base = gripper1.leftFinger.base.T * troty(0.1/Steps);
%                 gripper1.rightFinger.base = gripper1.rightFinger.base.T * troty(-0.1/Steps);
%                 gripper1.rightFinger.animate(gripper1.leftFinger.getpos);
%                 gripper1.leftFinger.animate(gripper1.rightFinger.getpos);
% 
%                 drawnow();
%             end
% 
% disp("Robot 1 gripper closed. Moving to final position")            
%             %Creep move slightly up
%             PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
%             Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.2,-0.18);
%             Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
%             jointTrajectory = jtraj(r1.model.getpos, Robot1Intermediate, Steps/2); %work out the path it takes
% 
%             %Move toward final position
%             PoseGuess = [deg2rad(20),   deg2rad(-110),   deg2rad(-110),   deg2rad(-145),   deg2rad(-80),   deg2rad(-5)];
%             StepPose = SE3(FinalObjectLocationsArray).T  * transl(0,0,0.15) * trotx(pi/2) * troty(pi/2);
%             Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
%             jointTrajectory2 = jtraj(jointTrajectory(25,:), Robot1Final, Steps); %work out the path it takes
% 
%             %Creep to final position
%             StepPose = SE3(FinalObjectLocationsArray).T  * transl(0,0,-1) * trotx(pi/2) * troty(pi/2);
%             Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
%             jointTrajectory3 = jtraj(jointTrajectory2(50,:), Robot1Final, Steps/2); %work out the path it takes
% 
%             jointTrajectory = [jointTrajectory; jointTrajectory2];
%             jtrajSize = size(jointTrajectory);
% 
%             %Move robot 1 to the final position
%             for i = 1:jtrajSize(1)
%                 %adjust and animate the postion of the gripper
%                 gripper1.gripperbase_.base = r1.model.fkineUTS(jointTrajectory(i,:)) * transl(0,0,-0.05);
%                 gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0) * troty(0.1);
%                 gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0) * troty(-0.1);
% 
%                 gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
%                 gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
%                 gripper1.rightFinger.animate(gripper1.rightFinger.getpos);
% 
%                 %Bottle_.model.base = r2.model.fkineUTS(jointTrajectory(i,:)) * troty(pi/2)  * transl(-0.18,0,-0.1)
% 
%                 Bottle_.model.base = r1.model.fkineUTS(jointTrajectory(i,:)) * trotx(-pi/2)  * transl(0,-0.08,-0.09);
%                 animate(Bottle_.model,jointTrajectory(i,:));
% 
%                 %animate the arm
%                 animate(r1.model,jointTrajectory(i,:));
% 
%                 drawnow();
% 
%             end
% r1.model.getpos
% disp("Final position reached. Opening gripper 1")  
% %%
% r1.model.getpos
% r1.model.teach(r1.model.getpos)
% 
% 
%             %%
%             % Open gripper
%             for i = 1:Steps 
%                 gripper1.rightFinger.base = gripper1.rightFinger.base.T * troty(0.2/Steps);
%                 gripper1.leftFinger.base = gripper1.leftFinger.base.T * troty(-0.2/Steps);
%                 gripper1.rightFinger.animate(gripper1.leftFinger.getpos);
%                 gripper1.leftFinger.animate(gripper1.rightFinger.getpos);
% 
%                 drawnow();
% 
%             end
% 
% 
% %%
% 
%             jointTrajectory = jtraj(r1.model.getpos, r1ResetPose, Steps); %work out the path it takes
%             %Move robot 1 out of the road
%             for i = 1:Steps
%                 %adjust and animate the postion of the gripper
%                 gripper1.gripperbase_.base = r1.model.fkineUTS(jointTrajectory(i,:)) * transl(0,0,-0.05);
%                 gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
%                 gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
% 
%                 gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
%                 gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
%                 gripper1.rightFinger.animate(gripper1.rightFinger.getpos);
% 
%                 %animate the arm
%                 animate(r1.model,jointTrajectory(i,:));
% 
%                 drawnow();
% 
%             end
% 
% 
%         end
% 
%         %% collisiondetection
% 
%         function CheckRobotReach(self)
%             %% 
%             r = r2; %The robot to evaluate
% 
%             checksize = true; %% change this to run the check for sizing
%             if checksize
%                 display('starting check on range');
% 
%                 R_qlim = r.model.qlim;
%                 stepRads = deg2rad(40);
% 
%                 pointCloudSize = prod(floor((R_qlim(1:6,2)-R_qlim(1:6,1))/stepRads + 1));
%                 pointCloud = zeros(pointCloudSize,3);
% 
%                 %Use an iterator that will be added to on each run
%                 counter = 1;
%                 tic
% 
%                 for q1 = R_qlim(1,1):stepRads:R_qlim(1,2)
%                     for q2 = R_qlim(2,1):stepRads:R_qlim(2,2)
%                         for q3 = R_qlim(3,1):stepRads:R_qlim(3,2)
%                             for q4 = R_qlim(4,1):stepRads:R_qlim(4,2)
%                                 for q5 = R_qlim(5,1):stepRads:R_qlim(5,2)
%                                     for q6 = R_qlim(6,1):stepRads:R_qlim(6,2)
%                                         %No q7 as final joint rotation not affecting final pose
%                                         q7 = 0;
%                                         NextPose = [q1,q2,q3,q4,q5,q6];
%                                         tr = r.model.fkineUTS(NextPose);
%                                         %If time permits, attempt using manual fkineUTS... 
% 
%                                         pointCloud(counter,:) = tr(1:3,4)';
%                                         counter = counter + 1;
%                                         if mod(counter/pointCloudSize * 100,1) == 0
%                                             display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudSize * 100),'% of poses']);
%                                         end
%                                     end
%                                 end
%                             end
%                         end
%                     end
%                 end
%             end
% 
%             %% Plotting the point cloud
%             if checksize
%                 rangePlot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
% 
%                 % convex hull and volume
%                 [convexHull, volume] = convhull(pointCloud, 'Simplify', true);
%                 convexHullPlot = trisurf(convexHull,pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'FaceColor', 'cyan');
%                 fprintf('- \nVolume: \n%gm^3\n-\n', volume);
% 
%                 drawnow();
% 
%                 % XYZ Max ranges
%                 Xmax = max(pointCloud(:,1)) - min(pointCloud(:,1));
%                 Ymax = max(pointCloud(:,2)) - min(pointCloud(:,2));
%                 Zmax = max(pointCloud(:,3)) - min(pointCloud(:,3));
%                 fprintf('- \nMax Range in X plane: %gm\nMax Range in Y plane: %gm\nMax Range in Z plane: %gm\n-\n', Xmax, Ymax, Zmax);
% 
%                 % max radius calculations
%                 Xrad = (abs((min(pointCloud(:,1))) + abs(max(pointCloud(:,1))))/2);
%                 Yrad = (abs((min(pointCloud(:,2))) + abs(max(pointCloud(:,2))))/2);
%                 Zrad = (abs((min(pointCloud(:,3))) + abs(max(pointCloud(:,3))))/2);
%                 fprintf('- \nRadius in X plane: %gm\nRadius in Y plane: %gm\nRadius in Z plane: %gm\n-\n', Xrad, Yrad, Zrad);
% 
%                 input('click enter to clear reaching plot')
% 
%                 delete(rangePlot);
%                 delete(convexHullPlot);
%             end
%         end
    
        end



    end
