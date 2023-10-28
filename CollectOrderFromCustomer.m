%% Move objects
function CollectOrderFromCustomer(Drink)
            %% 
            Steps = 50;
            %reduce the work by adding a guess
            PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];

            FingerRotationOpen = 0.1;
            FingerRotationClosed = -0.1;

            %input from GUI selects which drink should be ordered
            Drink = "Solo";
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
        
            % Use Robot 2 to move bottle from initial pose to intermediate pose
            % Robot close to position
            Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.1,0,-.24);
            robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps); %work out the path it takes

            % creep speed to get in close
            PoseGuess = [ -0.4049    0.8781   -0.6030   -1.4791   -1.1850    0.9801    0.6300];
            Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.1,0,-0.18);
            robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
            creepJointTrajectory = jtraj(jointTrajectory(50,:), robot2Bottle, Steps/2); %work out the path it takes

            jointTrajectory = [jointTrajectory; creepJointTrajectory ];
            jtrajSize = size(jointTrajectory);

            MoveRobot(r2, gripper2, false, jointTrajectory, NaN);

disp("Robot arrived at bottle, closing gripper")

            % Close gripper
            AlterGripper(gripper2, true); %false to open gripper, true to close

disp("Bottle picked up, moving to intermediate position")

            %creep Movement up after picking bottle
            Bottle_Pose = Bottle_.model.base.T * troty(-pi/2) * transl(0.2,0,-.3);
            robot2Bottle = r2.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory = jtraj(r2.model.getpos, robot2Bottle, Steps/2); %work out the path it takes

             %Movement to near intermediate position
            PoseGuess = [0.0420    1.3929   -1.4104   -0.9893    0.2391    0.8434    1.7315];
            StepPose = SE3(IntermediateObjectLocations).T * troty(-pi/2) * trotx(pi/2) * transl(0.14,0,-0.25);
            Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
            jointTrajectory2 = jtraj(jointTrajectory(25,:), Robot2Intermediate, Steps); %work out the path it takes

            %creep speed placement at intermediate position
            StepPose = SE3(IntermediateObjectLocations).T * troty(-pi/2) * trotx(pi/2) * transl(0.12,0,-0.18);
            Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
            jointTrajectory3 = jtraj(jointTrajectory2(50,:), Robot2Intermediate, Steps/2); %work out the path it takes

            jointTrajectory = [jointTrajectory; jointTrajectory2; jointTrajectory3];
            jtrajSize = size(jointTrajectory);

            MoveRobot(r2, gripper2, true, jointTrajectory,Bottle_);

disp("Bottle at intermediate position, opening grippper")

            %  Open gripper
            AlterGripper(gripper2, false); %false to open gripper, true to close


disp("Moving robot out of the road")
            % creep away from bottle
            PoseGuess = [0.0420    1.3929   -1.4104   -0.9893    0.2391    0.8434    1.7315];
            StepPose = SE3(IntermediateObjectLocations).T * troty(-pi/2) * trotx(pi/2) * transl(0.14,0,-0.25);
            Robot2Intermediate = r2.model.ikcon(StepPose,PoseGuess);
            jointTrajectory = jtraj(r2.model.getpos, Robot2Intermediate, Steps/2); %work out the path it takes

            % Move to reset pose
            StepPose = SE3(-1.15,0.09,1.502).T * troty(-pi/2) * trotx(pi/2) * transl(0.14,0,-0.25);
            Robot2Reset = r2.model.ikcon(StepPose,PoseGuess);
            jointTrajectory2 = jtraj(jointTrajectory(25,:), Robot2Reset, Steps); %work out the path it takes
            
            jointTrajectory = [jointTrajectory; jointTrajectory2];
            jtrajSize = size(jointTrajectory);

            MoveRobot(r2, gripper2, false, jointTrajectory, NaN);
% 
% holdPos = Bottle_.model.base;
% 
% Bottle_.model.base = holdPos;
% animate(Bottle_.model,Bottle_.model.getpos);

disp("Robot 1 moving into position")
            % Use Robot 1 to move bottle from intermediate pose to final pose
            PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
            Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.15,-0.11);
            Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory = jtraj(r1.model.getpos, Robot1Intermediate, Steps); %work out the path it takes

            %Creep to bottle position
            PoseGuess = [deg2rad(215) , deg2rad(-129) ,deg2rad(-80),deg2rad(210),deg2rad(-60),0];
            Bottle_Pose = Bottle_.model.base.T * trotx(pi/2) * troty(pi) * transl(0,0.09,-0.08);
            Robot1Intermediate = r1.model.ikcon(Bottle_Pose,PoseGuess);
            jointTrajectory2 = jtraj(jointTrajectory(50,:), Robot1Intermediate, Steps/2); %work out the path it takes

            jointTrajectory = [jointTrajectory; jointTrajectory2];
            jtrajSize = size(jointTrajectory);

            %Move robot A out of the road
            MoveRobot(r1, gripper1, false, jointTrajectory, NaN );
            
            
disp("Robot 1 in position. Gripper closing")

            % Close gripper
            AlterGripper(gripper1, true); %false to open gripper, true to close
            

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

            MoveRobot(r1, gripper1, true, jointTrajectory, Bottle_ );

           
disp("Final position reached. Opening gripper 1")  

            % Open gripper
            AlterGripper(gripper1, false); %false to open gripper, true to close

            %Get joint trajectory to destination
            jointTrajectory = jtraj(r1.model.getpos, r1ResetPose, Steps); %work out the path it takes
            
            MoveRobot(r1, gripper1, true, jointTrajectory, NaN );
end


  
function AlterGripper(gripper, gripperClosed)
    RotationY = 0.1;
    if gripperClosed
        RotationY = -0.1;
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
        BottleOffset =  trotx(-pi/2)  * transl(0,-0.08,-0.09);

        if robot.plyFileNameStem(1) == 'L'
            GripperOffset =  trotz(pi/2) * transl(0,0,0.05);
            BottleOffset =   troty(pi/2)  * transl(-0.18,0,-0.1);
        end

        bottlePassed = isa(bottle, 'IR_Object');
        
        jtrajSize = size(jTraj);
        for i = 1:jtrajSize
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

        