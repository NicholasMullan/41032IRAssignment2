%% 

%% Test the reset and rebuild from the main script



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
