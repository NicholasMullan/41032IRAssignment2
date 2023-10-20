% Connor Keogh      13220482
% Nicholas Mullan   13552361
% Patrick Hore      13623943

classdef Assignment2Group

    properties
        %% Set the initial position for each Object

        
        % Define the height of the bar / table here for use of robot placement
        BarTableheight = 0.74; %%(800mm) 
        OffsetTable = 0.02;

        %Work out how to set the bottle locations in bottle setup function
        InitialObjectLocationsArray =       [0,0,0];
        IntermediateObjectLocations =       [0,0,0];
        FinalObjectLocationsArray =         [0,0,0];

        NumberOfBottles = 9; %how many do we want to simulate
        %BottleArray = cell(NumberOfBottles, 1);
        BottleColours = {"Lemonade", "Coca-Cola", "Solo", "Fanta" }


    end
    
    

        methods
        function main(self)
            %% The main function for this setup
            ClearAndClose();
            self.SetupEnvironment();
            self.SetupRobots();
            self.SetArrayValues();
            self.PlaceBottles();
            % for i=0 : NumberOfBottles 
            %     CollectOrderFromCustomer(i);
            % end
        
        end
        
        function ClearAndClose()
            %% Reset simulation environment
            clc;
            clear all; 
            hold on
            % close all; %Dont close all. Dock the figure for ease of use
        end
        
        %% Set up Environment
        
        % initial object locations. 
        
        
        function SetupEnvironment()
        %% 
        
        clf; %Clear Current Figure
        clear all; %Clear all variables from workspace
        hold on;
        %Setting up enviroment 
        rs= 2; %Roomsize
        wh= 2; % Wall height
        wb=0; %wall base
        fh=0.01; %floor height
        % Create a textured surface for the floor.
         surf([-rs,-rs;rs,rs] ,[-rs,rs;-rs,rs] ,[fh,fh;fh,fh],'CData',imread('ConcreteFloor.jpg'),'FaceColor','texturemap')
        
        %
        % Define the axis limits for the plot.
        %axis([-rs, rs; -rs, rs; wb, wh]);
        %Setting Up Wall 1 
        % Define wallX, wallY, and wallZ arrays for the wall surface.
        
        
        % Create a textured surface for the wall.
        surf([-rs, -rs; rs, rs],[-rs, -rs; -rs, -rs],[wb, wh; wb, wh],'CData',imread('Brickwall.jpg'),'FaceColor','texturemap')
        
        %Setting Up Wall 2
        % Define wallX, wallY, and wallZ arrays for the wall surface.
        
        surf([-rs, -rs; -rs, -rs],[-rs, -rs; rs, rs],[wb, wh; wb, wh],'CData',imread('Brickwall.jpg'),'FaceColor','texturemap')
        
        
        
        % Place Evironment Alterations can be made in blender
        PlaceObject('Environment.ply',[-0.5,1,fh]);
        
        %Placing Human
        PlaceObject('personMaleOld.ply',[1,1,fh])
        
        
        %Place cctv camera
        cctvXYZ=[-rs-0.1,-rs,wh-0.5];
        %Placing CCTV
        PlaceObject('cctv.ply',cctvXYZ)
        
                
        end

        function SetupRobots()
        %% Creates the robots in their positions
        %Robot 1 is the UR5
        hold on

        r1 = UR3();
        r1.model.base = r1.model.base * SE3(-0.5,-0.3,BarTableheight + OffsetTable);
        r1.model.animate(r1.model.getpos);

        gripper1 = Gripper();
        gripper1.gripperbase_.base = r1.model.base.T * transl(0,0.70,-0.19) * troty(pi);
        gripper1.leftFinger.base = r1.model.base.T  * (gripper1.leftFinger.base.T * transl(0,0.70,-0.19)) * troty(pi);
        gripper1.rightFinger.base = r1.model.base.T * (gripper1.rightFinger.base.T * transl(0,0.70,-0.19)) * troty(pi);
        gripper1.gripperbase_.animate(gripper1.gripperbase_.getpos);
        gripper1.leftFinger.animate(gripper1.leftFinger.getpos);
        gripper1.rightFinger.animate(gripper1.rightFinger.getpos);
         
         
         
        %Robot 2 is custom robot
        hold on
        r2 = LinearLite6();
        %r2.model.base = r2.model.base * SE3(0,BarTableheight + OffsetTable,1.5);
        r2.model.base = r2.model.base *  SE3(-1.15,BarTableheight + OffsetTable,0);
        r2.model.animate(r2.model.getpos);

        gripper2 = Gripper();
        gripper2.gripperbase_.base = r1.model.base.T * transl(0,0.70,-0.19) * troty(pi);
        gripper2.leftFinger.base = r1.model.base.T  * (gripper2.leftFinger.base.T * transl(0,0.70,-0.19)) * troty(pi);
        gripper2.rightFinger.base = r1.model.base.T * (gripper2.rightFinger.base.T * transl(0,0.70,-0.19)) * troty(pi);
        gripper2.gripperbase_.animate(gripper2.gripperbase_.getpos);
        gripper2.leftFinger.animate(gripper2.leftFinger.getpos);
        gripper2.rightFinger.animate(gripper2.rightFinger.getpos);




        end
        
        function SetArrayValues()         
            %% Set up bottle initial locations
            % note no orientation considering bottle is mirrored about Z-axis.             
            TallBarTable = 0.95;

            InitialObjectLocationsArray = [
                %Location XYZ   
                -1.6,    -0.4,    TallBarTable;           %Object 1
                -1.6,    -0.2,   TallBarTable;          %Object 2
                -1.6,    0,   TallBarTable;          %Object 3
                -1.6,    0.2,    TallBarTable;          %Object 4
                -1.6,    0.4,   TallBarTable;         %Object 5
                -1.6,    0.6,   TallBarTable;         %Object 6
                -1.6,    0.8,    TallBarTable;          %Object 7
                -1.6,    1,   TallBarTable;         %Object 8
                -1.6,   1.2,   TallBarTable;         %Object 9
            ]
            
            % This may change to a single number
            IntermediateObjectLocations = [
                -1.1, -0.4,   BarTableheight;                 %Object 1
            ]
            
            %Setting up the code to be able to run 3 different orders for
            %the initial setup
            FinalObjectLocationsArray = [
                %Location XYZ   Rotation
                -0.3, -0.45,    BarTableheight;           %Object 1
                -0.163, -0.45,  BarTableheight;         %Object 2
                -0.026, -0.45,  BarTableheight;         %Object 3
            ];
        end

        function PlaceBottles(self)
            %% Use this to place each of the bottles in a position

            
            for i = 1:NumberOfBottles
                name = ['Bottle', num2str(i)];  
                BottleArray{i} = IR_Object('brick.ply',name, InitialObjectLocationsArray(i,:)) %no colon so we keep it as a property
            end

        end
    
        %% Move objects
        function CollectOrderFromCustomer(Num)
            %% 
            Steps = 50;
            %reduce the work by adding a guess
            PoseGuess = [-0.4, deg2rad(90), deg2rad(-45), deg2rad(-90), deg2rad(45),deg2rad(90), 0];

            %input from GUI selects which drink should be ordered
            Drink = "Lemonade";
            BottleNumber = 1;

            % Based on the drink choice, set the initial location in this
            % function
            switch (Drink)
                case "Lemonade"
                    BottleNumber = 1;
                case "Solo"
                    BottleNumber = 2;
                otherwise 
                    display("Error: " + Drink + " - out of stock");
            end

            BottleNumber = 8;

            %Find the bottle using its name and determine the relevant detai
            Bottle_ = BottleArray{length(BottleArray)-BottleNumber};
            Bottle_Pose = Bottle_.model.base.T * trotz(pi) * transl(0.2,0,0.1);
            display("Bottle found: " + Drink + " at position " + BottleNumber);
 
            %Now that we have the bottle, we can determine all the other
            %information through this
        
            % Use Robot 2 to move bottle from initial pose to intermediate pose
            robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps); %work out the path it takes

            for i = 1:Steps
                %adjust and animate the postion of the gripper
                

                %animate the arm
                animate(r2.model,jointTrajectory(i,:));
                
                drawnow();
        
            end
            
            disp("Robot arrived at order. Picking up now")
            %Now we pick up the item and use place it in the intermediate position

            % Close gripper
            for i = 1:Steps 
                gripper2.rightFinger.base = gripper2.rightFinger.base.T * troty(-0.2/Steps);
                gripper2.leftFinger.base = gripper2.leftFinger.base.T * troty(0.2/Steps);
                gripper2.rightFinger.animate(gripper2.leftFinger.getpos);
                gripper2.leftFinger.animate(gripper2.rightFinger.getpos);

                drawnow();

            end

            %update pose guess
            PoseGuess = [-0.4, deg2rad(90), deg2rad(-45), deg2rad(-90), deg2rad(45),deg2rad(90), 0];

            StepPose = SE3(-1.1, -0.4,   BarTableheight + 0.15);
            Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
            jointTrajectory2 = jtraj(r2.model.getpos, Robot2Intermediate, Steps); %work out the path it takes

            for i = 1:Steps
                %adjust and animate the postion of the gripper

                %animate the arm
                animate(r2.model,jointTrajectory2(i,:));
                
                drawnow();
        
            end

            % Open gripper
            for i = 1:Steps 
                gripper2.rightFinger.base = gripper2.rightFinger.base.T * troty(0.2/Steps);
                gripper2.leftFinger.base = gripper2.leftFinger.base.T * troty(-0.2/Steps);
                gripper2.rightFinger.animate(gripper2.leftFinger.getpos);
                gripper2.leftFinger.animate(gripper2.rightFinger.getpos);

                drawnow();

            end

            % Use Robot B to move bottle from intermediate pose to final pose
            % Close gripper
            for i = 1:Steps 
                gripper1.rightFinger.base = gripper1.rightFinger.base.T * troty(-0.2/Steps);
                gripper1.leftFinger.base = gripper1.leftFinger.base.T * troty(0.2/Steps);
                gripper1.rightFinger.animate(gripper1.leftFinger.getpos);
                gripper1.leftFinger.animate(gripper1.rightFinger.getpos);

                drawnow();

            end

            % Close gripper
            for i = 1:Steps 
                gripper1.rightFinger.base = gripper1.rightFinger.base.T * troty(0.2/Steps);
                gripper1.leftFinger.base = gripper1.leftFinger.base.T * troty(-0.2/Steps);
                gripper1.rightFinger.animate(gripper1.leftFinger.getpos);
                gripper1.leftFinger.animate(gripper1.rightFinger.getpos);

                drawnow();

            end
        
        
        
        end
        
        %% collisiondetection
        
        function CheckRobotReach(self)
            %% 
            r = r2; %The robot to evaluate

            checksize = true; %% change this to run the check for sizing
            if checksize
                display('starting check on range');
                
                R_qlim = r.model.qlim;
                stepRads = deg2rad(40);
                
                pointCloudSize = prod(floor((R_qlim(1:6,2)-R_qlim(1:6,1))/stepRads + 1));
                pointCloud = zeros(pointCloudSize,3);
               
                %Use an iterator that will be added to on each run
                counter = 1;
                tic
               
                for q1 = R_qlim(1,1):stepRads:R_qlim(1,2)
                    for q2 = R_qlim(2,1):stepRads:R_qlim(2,2)
                        for q3 = R_qlim(3,1):stepRads:R_qlim(3,2)
                            for q4 = R_qlim(4,1):stepRads:R_qlim(4,2)
                                for q5 = R_qlim(5,1):stepRads:R_qlim(5,2)
                                    for q6 = R_qlim(6,1):stepRads:R_qlim(6,2)
                                        %No q7 as final joint rotation not affecting final pose
                                        q7 = 0;
                                        NextPose = [q1,q2,q3,q4,q5,q6];
                                        tr = r.model.fkineUTS(NextPose);
                                        %If time permits, attempt using manual fkineUTS... 
                
                                        pointCloud(counter,:) = tr(1:3,4)';
                                        counter = counter + 1;
                                        if mod(counter/pointCloudSize * 100,1) == 0
                                            display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudSize * 100),'% of poses']);
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
                
            %% Plotting the point cloud
            if checksize
                rangePlot = plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
                
                % convex hull and volume
                [convexHull, volume] = convhull(pointCloud, 'Simplify', true);
                convexHullPlot = trisurf(convexHull,pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'FaceColor', 'cyan');
                fprintf('- \nVolume: \n%gm^3\n-\n', volume);
                
                drawnow();
                
                % XYZ Max ranges
                Xmax = max(pointCloud(:,1)) - min(pointCloud(:,1));
                Ymax = max(pointCloud(:,2)) - min(pointCloud(:,2));
                Zmax = max(pointCloud(:,3)) - min(pointCloud(:,3));
                fprintf('- \nMax Range in X plane: %gm\nMax Range in Y plane: %gm\nMax Range in Z plane: %gm\n-\n', Xmax, Ymax, Zmax);
                
                % max radius calculations
                Xrad = (abs((min(pointCloud(:,1))) + abs(max(pointCloud(:,1))))/2);
                Yrad = (abs((min(pointCloud(:,2))) + abs(max(pointCloud(:,2))))/2);
                Zrad = (abs((min(pointCloud(:,3))) + abs(max(pointCloud(:,3))))/2);
                fprintf('- \nRadius in X plane: %gm\nRadius in Y plane: %gm\nRadius in Z plane: %gm\n-\n', Xrad, Yrad, Zrad);
                
                input('click enter to clear reaching plot')
                
                delete(rangePlot);
                delete(convexHullPlot);
            end
        end
    
        end



    end
