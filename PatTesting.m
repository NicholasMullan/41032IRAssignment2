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
        
                
      

%% testing light curtain


input('\nlight curtain demo')

[x,z] = meshgrid(-1.5:0.01:1.5, 0:0.01:1);  %setting location of meshgrid
y(1:size(x,1),1:1) = -0.5;
lightCurtainS1 = surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');

[x,z] = meshgrid(-1.5:0.01:1.5, 0:0.01:1);  %setting location of meshgrid
y(1:size(x,1),1:1) = 0.5;
lightCurtainS2 = surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');

[f,v,data] = plyread('personMaleOld.ply','tri');
kidVertices = v;

input('\nMove kid into light curtain')

kid.man.base = transl(-1,-0.5,0);
kid.man.animate(kid.man.getpos);
pause(0.01);

kidVertices(:,1) = kidVertices(:,1) + kid.man.base(1,4);
kidVertices(:,2) = kidVertices(:,2) + kid.man.base(2,4);
kidVertices(:,3) = kidVertices(:,3) + kid.man.base(3,4);

if max(kidVertices(:,2)) >= -0.5
    fprintf("Light Curtain has been activated")
    

end
