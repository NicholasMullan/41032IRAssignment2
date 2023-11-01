%% 
%%
Assignment = Assignment2Group()


%%
Assignment.CollectOrderFromCustomer('Solo');
Assignment.CollectOrderFromCustomer('CocaCola');
Assignment.CollectOrderFromCustomer('Lemonade');
Assignment.CollectOrderFromCustomer('Fanta');


%%
Assignment.r1.model.getpos
Assignment.r1.model.teach(Assignment.r1.model.getpos);

%% 
Assignment.r2.model.getpos
Assignment.r2.model.teach(Assignment.r2.model.getpos);


%% 
Assignment.Bottle_.model.base = Assignment.Bottle_.BasePose;
Assignment.Bottle_.model.animate(Assignment.Bottle_.model.getpos);

Assignment.EmptyCan.model.base = SE3(Assignment.FinalObjectLocationsArray(1,:)); 
Assignment.EmptyCan.model.animate(Assignment.EmptyCan.model.getpos);



Assignment.StateMachine = 0; %Empty collected

Assignment.Bottle_.vertexColours = Assignment.Bottle_.InitialVertexColours;
Assignment.Bottle_.h.link(2).Children.FaceVertexCData = Assignment.Bottle_.vertexColours;
Assignment.Bottle_.h.link(2).Children.FaceColor = 'interp';
drawnow();

Assignment.EmptyCan.vertexColours = Assignment.EmptyCan.InitialVertexColours;
Assignment.EmptyCan.h.link(2).Children.FaceVertexCData = Assignment.EmptyCan.vertexColours;
Assignment.EmptyCan.h.link(2).Children.FaceColor = 'interp';
drawnow();
display("Bottles replaced and scene reset.");


%%
Assignment.r2ResetPose = [0,     -pi/2,     0,     0,     0,     0,    0];
Assignment.r1ResetPose = [0,     -pi/2,     0,     0,     0,     0];


%% Testing final step
tempPose = Assignment.r1.model.getpos;
cupTempPose = Assignment.EmptyCan.model.base;

%%

Assignment.EmptyCan.model.base = cupTempPose.T;
Assignment.SubStateMachine = 0;
Assignment.StateMachine = 3; %Empty collected


%%Testing emptying a bottle
FullBottle = Assignment.BottleArray{3};
EmptyBottle = Assignment.BottleArray{1};


Array1Size = size(FullBottle.vertexColours)
Array2Size = size(EmptyBottle.vertexColours)
Steps = 100

Avg1 = mean(FullBottle.vertexColours(:,1));
Avg2 = mean(FullBottle.vertexColours(:,1));
Avg3 = mean(FullBottle.vertexColours(:,3));
colour = [Avg1, Avg2, Avg3]

for i = 1:Steps
     %Only change 98% of the values
    fluidHeight = round(((i / Steps) * Array1Size(1)),0)

    for vertexIdx = 1:fluidHeight
        FullBottle.vertexColours(vertexIdx,:) = [1,1,1];
        EmptyBottle.vertexColours(Array2Size(1) - vertexIdx, :) = colour;
        % Set the color to red for this vertex
    end
    FullBottle.h.link(2).Children.FaceVertexCData = FullBottle.vertexColours;
    FullBottle.h.link(2).Children.FaceColor = 'interp';

    EmptyBottle.h.link(2).Children.FaceVertexCData = EmptyBottle.vertexColours;
    EmptyBottle.h.link(2).Children.FaceColor = 'interp';

     % Update the plot to visualize the changes
    drawnow();
    pause(0.005);
end















%% 
Steps = 50;
%reduce the work by adding a guess
PoseGuess = [-0.4145, 1.0969, -0.5204, -1.6201, -1.3009, 1.1764, 0.6240];

FingerRotationOpen = 0.1;
FingerRotationClosed = -0.1;

%input from GUI selects which drink should be ordered
Drink = "Solo";
BottleNumber = 1;

Bottle_ = BottleArray{length(BottleArray)-BottleNumber+1};
display("Bottle found: " + Drink + " at position " + BottleNumber);

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






























%% Test the creation of bottles
bottleTypes = {
                'CanYellow.ply';
                'CanRed.ply';
                'CanBlue.ply';
                'CanGreen.ply';
                };

StartingX = 0;
TallBarTable = 0.05;
InitialObjectLocationsArray = [
                %Location XYZ   
                StartingX,    -0.4,    TallBarTable;           %Object 1
                StartingX,    -0.2,   TallBarTable;          %Object 2
                StartingX,    0,   TallBarTable;          %Object 3
                StartingX,    0.2,    TallBarTable;          %Object 4
            ];

for i = 1:size(bottleTypes,1)
    BottleArray{i} = IR_Object([bottleTypes{i}],['Bottle', num2str(i)], InitialObjectLocationsArray(i,:)); 
end



%% 
clc;
clf;
clear all;

%% 
hold on

name = 'CanYellow.ply';
%plydata_ = plyread(name);
%[faceData, vertexData] = plyread(name, 'tri');


[faceData, vertexData, plyData{1}] = plyread([name], 'tri');
%vertex_colors = [plydata.vertex.red, plydata.vertex.green, plydata.vertex.blue];

vertexColours = [plyData{1}.vertex.red ...
                 , plyData{1}.vertex.green ...
                 , plyData{1}.vertex.blue]/255;
                    

link1 = Link('alpha',0,'a',0,'d',0,'offset',0)
model1 = SerialLink(link1,'name','test');

%link1.Children.FaceVertexCData = vertexColours;

%axis equal;
%patch('Vertices', vertexData, 'Faces', faceData, 'FaceVertexCData', vertex_colors, ...
    %'FaceColor', 'flat', 'EdgeColor', 'none');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Colored PLY Object');
% view(3);


%% Test room creation
        rs = 2; % RoomSize
        surf([-rs,-rs;rs,rs] ,[-rs,rs;-rs,rs] ,[0.01,0.01;0.01,0.01] ,'CData',imread('concrete.jpg') ,'FaceColor','texturemap');

        %put in walls
        %TopWall = PlaceObject('wall.ply',[0,2,1]);
        %LeftWall = PlaceObject('wall.ply',[0,2,1]);
        %rotate(LeftWall, [0,0,1], 90, [0,0,0]);

        %surf([-1.375,-1.375;-1.25,-1.25],[-1.25,1.25;-1.25,1.25],[0.02,0.02;0.02,0.02],'CData',imread('tape.jpg'),'FaceColor','texturemap');
        %surf([1.375,1.375;1.25,1.25],[-1.25,1.25;-1.25,1.25],[0.02,0.02;0.02,0.02],'CData',imread('tape.jpg'),'FaceColor','texturemap');
        %surf([-1.375,1.375;-1.375,1.375],[-1.375,-1.375;-1.25,-1.25],[0.02,0.02;0.02,0.02],'CData',imread('tape.jpg'),'FaceColor','texturemap');
        %surf([-1.375,1.375;-1.375,1.375],[1.375,1.375;1.25,1.25],[0.02,0.02;0.02,0.02],'CData',imread('tape.jpg'),'FaceColor','texturemap');

        %PlaceObject('tableBrown2.1x1.4x0.5m.ply',[0,0,0]);
        %PlaceObject('Shelves.ply',[0,0,0]);
