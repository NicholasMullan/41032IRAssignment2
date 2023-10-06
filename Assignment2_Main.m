% Connor Keogh      13220482
% Nicholas Mullan   11111111
% Patrick Hore      11111111

classdef Assignment2Group

    properties
        %% Set the initial position for each Object
        NumberOfObjects = 9; 
        
        % Define the height of the bar / table here for use of robot placement
        BarTableheight = 0.5; 

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
        %%     
        
        
        
        
        
        
        end
        
        function SetArrayValues()         
            %% Set up bottle initial locations
            % note no orientation considering bottle is mirrored about Z-axis.             
            InitialObjectLocationsArray = [
                %Location XYZ   
                -0.32, 0.45,    self.BarTableheight;           %Object 1
                -0.163, 0.45,   self.BarTableheight;          %Object 2
                -0.026, 0.45,   self.BarTableheight;          %Object 3
                -0.32, 0.45,    self.BarTableheight;          %Object 4
                -0.163, 0.45,   self.BarTableheight;         %Object 5
                -0.026, 0.45,   self.BarTableheight;         %Object 6
                -0.32, 0.45,    self.BarTableheight;          %Object 7
                -0.163, 0.45,   self.BarTableheight;         %Object 8
                -0.026, 0.45,   self.BarTableheight;         %Object 9
            ]
            
            % This may change to a single number
            IntermediateObjectLocationsArray = [
                0, 0.45,        self.BarTableheight;                 %Object 1
                0, 0.45,        self.BarTableheight;                 %Object 2
                0, 0.45,        self.BarTableheight;                 %Object 3
                0, 0.45,        self.BarTableheight;                 %Object 4
                0, 0.45,        self.BarTableheight;                 %Object 5
                0, 0.45,        self.BarTableheight;                 %Object 6
                0, 0.45,        self.BarTableheight;                 %Object 7
                0, 0.45,        self.BarTableheight;                 %Object 8
                0, 0.45,        self.BarTableheight;                 %Object 9
            
            ]
            
            %The aim is to arrange the Objects end-to-end, forming 3 rows, with each
            %row accommodating 3 Objects. This arrangement culminates in a stacked configuration that adheres to these
            %specifications
            FinalObjectLocationsArray = [
                %Location XYZ   Rotation
                -0.3, -0.45,    self.BarTableheight;           %Object 1
                -0.163, -0.45,  self.BarTableheight;         %Object 2
                -0.026, -0.45,  self.BarTableheight;         %Object 3
                -0.3, -0.45,    self.BarTableheight;          %Object 4
                -0.163, -0.45,  self.BarTableheight;        %Object 5
                -0.026, -0.45,  self.BarTableheight;        %Object 6
                -0.3, -0.45,    self.BarTableheight;          %Object 7
                -0.163, -0.45,  self.BarTableheight;        %Object 8
                -0.026, -0.45,  self.BarTableheight;        %Object 9
            ]
        end
    
        %% Move objects
        function CollectOrderFromCustomer(Num)
            
            NumberOfCustomers = 3; %Start with 3 serves to start with and develop up to 9 in the future if need be. 
        
            Steps = 50;
            PoseGuess = [-0.4, deg2rad(90), deg2rad(-45), deg2rad(-90), deg2rad(45),deg2rad(90), 0];
        
            
            %Trigger the customer to appear at an offset from the final location
             
        
        
            % Use Robot A to move bottle from initial pose to intermediate pose
        
        
            % Use Robot B to move bottle from intermediate pose to final pose
        
        
        
        end
        
        %% collisiondetection
        
        
        
    
        end



    end
