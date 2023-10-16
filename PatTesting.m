clf; %Clear Current Figure
clear all; %Clear all variables from workspace
hold on;
%Setting up enviroment 
rs= 3; %Roomsize
wh= 3; % Wall height
wb=0; %wall base

% Create a textured surface for the floor.
 surf([-rs,-rs;rs,rs] ,[-rs,rs;-rs,rs] ,[0.01,0.01;0.01,0.01],'CData',imread('ConcreteFloor.jpg'),'FaceColor','texturemap')

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

%'table'
table1XYZ = [0,0,0];
% Call the PlaceObject function to place the table.
PlaceObject('tableBrown2.1x1.4x0.5m.ply',table1XYZ);

%'table'
table2XYZ = [2,1,0];
% Call the PlaceObject function to place the table.
PlaceObject('tableBrown2.1x1.4x0.5m.ply',table2XYZ);
