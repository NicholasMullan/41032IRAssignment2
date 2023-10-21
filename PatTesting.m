%% Set up Environment
        
        % initial object locations. 
        
        
       
        
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
        HumanXYZ=[1,2,fh];
        PlaceObject('personMaleOld.ply',HumanXYZ)
        
        
        %Place cctv camera
        cctvXYZ=[-rs-0.1,-rs,wh-0.5];
        %Placing CCTV
        PlaceObject('cctv.ply',cctvXYZ)
        
                
      


        % testing light curtain


input('\nlight curtain demo')
%light curtain 1
[y,z] = meshgrid(-2:0.01:2, 0:0.01:2);  %setting location of meshgrid
x = -0.12 * ones(size(y));

lightCurtainS1 = surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');
%Light curtain 2
[y,z] = meshgrid(-2:0.01:2, 0:0.01:2);  %setting location of meshgrid
x = 0.12 * ones(size(y));

lightCurtainS2 = surf(x,y,z,'FaceAlpha',0.1,'EdgeColor','none');

[f,v,data] = plyread('personMaleOld.ply','tri');
ManVertices = v;

input('\nMove Man into light curtain')

%Intial Man Position
InitalHumanPosition=HumanXYZ;
HumanTransform = transl(InitalHumanPosition);
%HumanTransform.animate(InitalHumanPosition.getpos)
%kid.man.base = transl(-1,-0.5,0);
%kid.man.animate(kid.man.getpos);
pause(0.01);

ManVertices(:,1) = ManVertices(:,1) + HumanTransform(1,4);
ManVertices(:,2) = ManVertices(:,2) + HumanTransform(2,4);
ManVertices(:,3) = ManVertices(:,3) + HumanTransform(3,4);

if max(ManVertices(:,2)) >= -0.5
    fprintf("Light Curtain has been activated")
    

end