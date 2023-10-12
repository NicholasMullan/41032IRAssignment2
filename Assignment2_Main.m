% Connor Keogh      13220482
% Nicholas Mullan   11111111
% Patrick Hore      13623943

classdef Assignment2Group

    properties
        %% Set the initial position for each Object
        NumberOfObjects = 9; 
        
        % Define the height of the bar / table here for use of robot placement
        BarTableheight = 0.9; %%(900mm) 

        %Work out how to set these
        InitialObjectLocationsArray =       [0,0,0];
        IntermediateObjectLocationsArray=   [0,0,0];
        FinalObjectLocationsArray =         [0,0,0];

    end
    
    

        methods
        function main()
            ClearAndClose(self);
            SetupEnvironment(self);
            SetArrayValues(self);
            for i=0 : NumberOfObjects 
                CollectOrderFromCustomer(i);
            end
        
        end
        
        function ClearAndClose()
            %% Reset simulation environment
            clc;
            clf; 
            clear all; 
            % close all; %Dont close all. Dock the figure for ease of use
        end
        
        %% Set up Environment
        
        % initial object locations. 
        
        
        function SetupEnvironment()
       
%% ENVIRONMENT SET UP
%Setting up enviroment 
% Define the ENVIRONMENTXYZ position.
ENVIRONMENTXYZ = [ 0,0,0];
% Call the PlaceObject function to place the table.
PlaceObject('ENVIRONMENT.ply',ENVIRONMENTXYZ);
        
        
        end
        
        function SetArrayValues()         
            %% Set up bottle initial locations
            % note no orientation considering bottle is mirrored about Z-axis.             
            InitialObjectLocationsArray = [
                %Location XYZ   
                -0.32, 0.45,    BarTableheight;           %Object 1
                -0.163, 0.45,   BarTableheight;          %Object 2
                -0.026, 0.45,   BarTableheight;          %Object 3
                -0.32, 0.45,    BarTableheight;          %Object 4
                -0.163, 0.45,   BarTableheight;         %Object 5
                -0.026, 0.45,   BarTableheight;         %Object 6
                -0.32, 0.45,    BarTableheight;          %Object 7
                -0.163, 0.45,   BarTableheight;         %Object 8
                -0.026, 0.45,   BarTableheight;         %Object 9
            ]
            
            % This may change to a single number
            IntermediateObjectLocationsArray = [
                0, 0.45,        BarTableheight;                 %Object 1
                0, 0.45,        BarTableheight;                 %Object 2
                0, 0.45,        BarTableheight;                 %Object 3
                0, 0.45,        BarTableheight;                 %Object 4
                0, 0.45,        BarTableheight;                 %Object 5
                0, 0.45,        BarTableheight;                 %Object 6
                0, 0.45,        BarTableheight;                 %Object 7
                0, 0.45,        BarTableheight;                 %Object 8
                0, 0.45,        BarTableheight;                 %Object 9
            
            ]
            
            %Setting up the code to be able to run 3 different orders for
            %the initial setup
            FinalObjectLocationsArray = [
                %Location XYZ   Rotation
                -0.3, -0.45,    BarTableheight;           %Object 1
                -0.163, -0.45,  BarTableheight;         %Object 2
                -0.026, -0.45,  BarTableheight;         %Object 3
            ]
        end
    
        %% Move objects
        function CollectOrderFromCustomer(Num)
            %% 
            Steps = 50;
            PoseGuess = [-0.4, deg2rad(90), deg2rad(-45), deg2rad(-90), deg2rad(45),deg2rad(90), 0];
            
            %input from GUI selects which drink should be ordered
            Drink = "Red";
            InitialLocation = 1;

            % Based on the drink choice, set the initial location in this
            % function
            switch (Drink)
                case "Red"
                    InitialLocation = 1;
                otherwise 
                    display("Error: " + Drink + " - out of stock");
            end

            %Trigger the customer to appear at an offset from the final location
            BottleLocation = InitialObjectLocationsArray(InitialLocation)
        
        
            % Use Robot A to move bottle from initial pose to intermediate pose



        
            % Use Robot B to move bottle from intermediate pose to final pose
        
        
        
        end
        
        %% collisiondetection
        
        
        
    
        end



    end
