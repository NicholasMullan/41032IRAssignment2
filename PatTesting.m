% %% Set up Environment
% 
%         % initial object locations. 
% 
% 
% 
% 
%         clf; %Clear Current Figure
%         clear all; %Clear all variables from workspace
%         hold on;
%         %Setting up enviroment 
%         rs= 2; %Roomsize
%         wh= 2; % Wall height
%         wb=0; %wall base
%         fh=0.01; %floor height
%         % Create a textured surface for the floor.
%          surf([-rs,-rs;rs,rs] ,[-rs,rs;-rs,rs] ,[fh,fh;fh,fh],'CData',imread('ConcreteFloor.jpg'),'FaceColor','texturemap')
% 
%         %
%         % Define the axis limits for the plot.
%         %axis([-rs, rs; -rs, rs; wb, wh]);
%         %Setting Up Wall 1 
%         % Define wallX, wallY, and wallZ arrays for the wall surface.
% 
% 
%         % Create a textured surface for the wall.
%         surf([-rs, -rs; rs, rs],[-rs, -rs; -rs, -rs],[wb, wh; wb, wh],'CData',imread('Brickwall.jpg'),'FaceColor','texturemap')
% 
%         %Setting Up Wall 2
%         % Define wallX, wallY, and wallZ arrays for the wall surface.
% 
%         surf([-rs, -rs; -rs, -rs],[-rs, -rs; rs, rs],[wb, wh; wb, wh],'CData',imread('Brickwall.jpg'),'FaceColor','texturemap')
% 
% 
% 
%         % Place Evironment Alterations can be made in blender
%         PlaceObject('Environment.ply',[-0.5,1,fh]);
% 
%         %Placing Human
%         HumanXYZ=[1,2,fh];
%         PlaceObject('personMaleOld.ply',HumanXYZ)
% 
% 
%         %Place cctv camera
%         cctvXYZ=[-rs+0.1,-rs,wh-0.5];
%         %Placing CCTV
%         PlaceObject('cctv.ply',cctvXYZ)
% 
% 
% 
% 
% 
%         %% testing light curtain
% 
% 
% input('\nlight curtain demo')
% %light curtain 1
% [y,z] = meshgrid(-2:0.01:2, 0:0.01:2);  %setting location of meshgrid
% x = -0.12 * ones(size(y));
% 
% lightCurtainS1 = surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');
% %Light curtain 2
% [y,z] = meshgrid(-2:0.01:2, 0:0.01:2);  %setting location of meshgrid
% x = 0.12 * ones(size(y));
% 
% lightCurtainS2 = surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');
% 
% [f,v,data] = plyread('personMaleOld.ply','tri');
% ManVertices = v;
% 
% input('\nMove Man into light curtain')
% 
% %Intial Man Position
% InitalHumanPosition=HumanXYZ;
% HumanTransform = transl(InitalHumanPosition);
% %HumanTransform.animate(InitalHumanPosition.getpos)
% %kid.man.base = transl(-1,-0.5,0);
% %kid.man.animate(kid.man.getpos);
% pause(0.01);
% 
% ManVertices(:,1) = ManVertices(:,1) + HumanTransform(1,4);
% ManVertices(:,2) = ManVertices(:,2) + HumanTransform(2,4);
% ManVertices(:,3) = ManVertices(:,3) + HumanTransform(3,4);
% 
% if max(ManVertices(:,2)) >= -0.5
%     fprintf("Light Curtain has been activated")
% 
% 
% end
% %%  Define the available bottle colors
%      hold on;
% 
% PlaceObject ('CanBlue.ply',[1,1,1])
% plyread('CanBlue.ply')
% PlaceObject ('CanRed.ply',[2,1,1]);
% PlaceObject ('CanYellow.ply',[3,1,1]);
% PlaceObject ('CanGreen.ply',[4,1,1]);
%% degrees to rad
a = deg2rad(0)
b = deg2rad(-35)
c = deg2rad(70)
d = deg2rad(-30)
e = deg2rad(90)

%% Test Ur3 Script
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
    
        % GUI Opened
        gui = GUI();

        %% Move objects
        function CollectOrderFromCustomer(Num)
            %% 
            Steps = 50;
            %reduce the work by adding a guess
            PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];

            %input from GUI selects which drink should be ordered
            Drink = "Fanta";
            % BottleNumber = 2;

            % Based on the drink choice, set the initial location in this
            % function
            switch (Drink)
                case "Lemonade"
                    BottleNumber = 1;
                case "Solo"
                    BottleNumber = 2;
                case "Fanta"
                    BottleNumber = 3;
                case "CocaCola"
                    BottleNumber = 4;
                otherwise 
                    display("Error: " + Drink + " - out of stock");
            end
                        %BottleNumber = 1;


            %Find the bottle using its name and determine the relevant detai
            Bottle_ = BottleArray{length(BottleArray)-BottleNumber+1};


            display("Bottle found: " + Drink + " at position " + BottleNumber);

            %Now that we have the bottle, we can determine all the other
            %information through this




            %holdPos = Bottle_.model.base;

%Bottle_.model.base = holdPos;
%animate(Bottle_.model,Bottle_.model.getpos);

disp("Robot 1 moving into position")
            % Use Robot 1 to move bottle from intermediate pose to final pose
            PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
            Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.15,-0.11);
            Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory = jtraj(r1.model.getpos, Robot1Intermediate, Steps); %work out the path it takes

            %Creep to bottle position
            PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
            Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.1,-0.1);
            Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory2 = jtraj(jointTrajectory(50,:), Robot1Intermediate, Steps/2); %work out the path it takes

            jointTrajectory = [jointTrajectory; jointTrajectory2];
            jtrajSize = size(jointTrajectory);

            %Move robot A out of the road
            for i = 1:jtrajSize
                %adjust and animate the postion of the gripper
                gripper1.gripperbase_.base = r1.model.fkineUTS(jointTrajectory(i,:)) * transl(0,0,-0.05);
                gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
                gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);

                gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
                gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
                gripper1.rightFinger.animate(gripper1.rightFinger.getpos);

                %animate the arm
                animate(r1.model,jointTrajectory(i,:));

                drawnow();

            end
disp("Robot 1 in position. Gripper closing")
r1.model.getpos
            % Close gripper
            for i = 1:Steps 
                gripper1.leftFinger.base = gripper1.leftFinger.base.T * troty(0.1/Steps);
                gripper1.rightFinger.base = gripper1.rightFinger.base.T * troty(-0.1/Steps);
                gripper1.rightFinger.animate(gripper1.leftFinger.getpos);
                gripper1.leftFinger.animate(gripper1.rightFinger.getpos);

                drawnow();
            end

disp("Robot 1 gripper closed. Moving to final position")            
            %Creep move slightly up
            PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
            Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.2,-0.18);
            Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory = jtraj(r1.model.getpos, Robot1Intermediate, Steps/2); %work out the path it takes

            %Move toward final position
            PoseGuess = [deg2rad(20),   deg2rad(-110),   deg2rad(-110),   deg2rad(-145),   deg2rad(-80),   deg2rad(-5)];
            StepPose = SE3(FinalObjectLocationsArray).T  * transl(0,0,0.15) * trotx(pi/2) * troty(pi/2);
            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
            jointTrajectory2 = jtraj(jointTrajectory(25,:), Robot1Final, Steps); %work out the path it takes

            %Creep to final position
            StepPose = SE3(FinalObjectLocationsArray).T  * transl(0,0,-1) * trotx(pi/2) * troty(pi/2);
            Robot1Final = r1.model.ikcon(StepPose,PoseGuess);
            jointTrajectory3 = jtraj(jointTrajectory2(50,:), Robot1Final, Steps/2); %work out the path it takes

            jointTrajectory = [jointTrajectory; jointTrajectory2];
            jtrajSize = size(jointTrajectory);

            %Move robot 1 to the final position
            for i = 1:jtrajSize(1)
                %adjust and animate the postion of the gripper
                gripper1.gripperbase_.base = r1.model.fkineUTS(jointTrajectory(i,:)) * transl(0,0,-0.05);
                gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0) * troty(0.1);
                gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0) * troty(-0.1);

                gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
                gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
                gripper1.rightFinger.animate(gripper1.rightFinger.getpos);

                %Bottle_.model.base = r2.model.fkineUTS(jointTrajectory(i,:)) * troty(pi/2)  * transl(-0.18,0,-0.1)

                Bottle_.model.base = r1.model.fkineUTS(jointTrajectory(i,:)) * trotx(-pi/2)  * transl(0,-0.08,-0.09);
                animate(Bottle_.model,jointTrajectory(i,:));

                %animate the arm
                animate(r1.model,jointTrajectory(i,:));

                drawnow();

            end
r1.model.getpos
disp("Final position reached. Opening gripper 1")  



            
            % Open gripper
            for i = 1:Steps 
                gripper1.rightFinger.base = gripper1.rightFinger.base.T * troty(0.2/Steps);
                gripper1.leftFinger.base = gripper1.leftFinger.base.T * troty(-0.2/Steps);
                gripper1.rightFinger.animate(gripper1.leftFinger.getpos);
                gripper1.leftFinger.animate(gripper1.rightFinger.getpos);

                drawnow();

            end




            jointTrajectory = jtraj(r1.model.getpos, r1ResetPose, Steps); %work out the path it takes
            %Move robot 1 out of the road
            for i = 1:Steps
                %adjust and animate the postion of the gripper
                gripper1.gripperbase_.base = r1.model.fkineUTS(jointTrajectory(i,:)) * transl(0,0,-0.05);
                gripper1.leftFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);
                gripper1.rightFinger.base = gripper1.gripperbase_.base.T * transl(0,0.1,0);

                gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
                gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
                gripper1.rightFinger.animate(gripper1.rightFinger.getpos);

                %animate the arm
                animate(r1.model,jointTrajectory(i,:));

                drawnow();

            end


        end

        



    end
