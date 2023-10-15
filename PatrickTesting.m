%%
clf; %Clear Current Figure
clear all; %Clear all variables from workspace
clc %clear command window
hold on;
%Setting up enviroment 
x1 = -2; % X-coordinate start
x2 = 2; % X-coordinate end
y1 = -2; % Y-coordinate start
y2 = 2; % Y-coordinate end
z1 = 0; % Z-coordinate start
z2 = 3; % Z-coordinate end
% Define the ENVIRONMENTXYZ position.
ENVIROMENTBase = transl(0, 0, 0) * rpy2tr(0, 0, 0); % Sets default value
ENVIRONMENTRad = 3;
ENVIRONMENTHeight = 3;
ENVIRONMENTXYZ = [ 0,0,0];
% Call the PlaceObject function to place the table.
PlaceObject('TESTENVIRONEMT.ply',ENVIRONMENTXYZ);
%Loading the FLoor
floorX = [ENVIROMENTBase(1,4)-ENVIRONMENTRad,ENVIROMENTBase(1,4)-ENVIRONMENTRad;ENVIROMENTBase(1,4)+ENVIRONMENTRad,ENVIROMENTBase(1,4)+ENVIRONMENTRad];
floorY = [ENVIROMENTBase(2,4)-ENVIRONMENTRad,ENVIROMENTBase(2, 4)+ENVIRONMENTRad;ENVIROMENTBase(2,4)-ENVIRONMENTRad,ENVIROMENTBase(2,4)+ENVIRONMENTRad];
floorZ = [ENVIROMENTBase(3,4),ENVIROMENTBase(3,4);ENVIROMENTBase(3,4),ENVIROMENTBase(3,4)];

% Create a textured surface for the floor.
surf(floorX,floorY,floorZ,'CData',imread('ConcreteFloor.jpg'),'FaceColor','texturemap');
% Define the axis limits for the plot.
axis([ENVIROMENTBase(1, 4) - ENVIRONMENTRad,ENVIROMENTBase(1, 4) + ENVIRONMENTRad, ENVIROMENTBase(2, 4) - ENVIRONMENTRad,ENVIROMENTBase(2, 4) + ENVIRONMENTRad, ENVIROMENTBase(3, 4),ENVIROMENTBase(3, 4) + ENVIRONMENTHeight]);

%Setting Up Wall 1 
% Define wallX, wallY, and wallZ arrays for the wall surface.
wall1X = [x1, x2; x1, x2]; % Specify x1 and x2 as the X-coordinates.
wall1Y = [-2, -2; -2, -2]; % Specify y1 and y2 as the Y-coordinates.
wall1Z = [z1, z1; z2, z2]; % Specify z1 and z2 as the Z-coordinates.

% Create a textured surface for the wall.
surf(wall1X,wall1Y,wall1Z,'CData',imread('Brickwall.jpg'),'FaceColor','texturemap');

%Setting Up Wall 2
% Define wallX, wallY, and wallZ arrays for the wall surface.
wall2X = [-1.5, -1.5; -1.5, -1.5]; % Specify x1 and x2 as the X-coordinates.
wall2Y = [y1, y2; y1, y2]; % Specify y1 and y2 as the Y-coordinates.
wall2Z = [z1, z1; z2, z2]; % Specify z1 and z2 as the Z-coordinates.

surf(wall2X,wall2Y,wall2Z,'CData',imread('Brickwall.jpg'),'FaceColor','texturemap');