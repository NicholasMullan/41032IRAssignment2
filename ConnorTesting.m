%% 

%% Test the reset and rebuild from the main script



%% Test the creation of arrays


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
