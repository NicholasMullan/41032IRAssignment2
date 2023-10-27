% Connor Keogh      13220482
% Nicholas Mullan   13552361
% Patrick Hore      13623943

classdef Assignment2Group

    properties
        %% Set the initial position for each Object

        % Define the height of the bar / table here for use of robot placement
        BarTableheight = 0.74; %%(800mm) 
        TallBarTable = 0.95;
        OffsetTable = 0.02;


        bottleTypes = {
                'CanYellow.ply';
                'CanRed.ply';
                'CanBlue.ply';
                'CanGreen.ply';
                };
        arraySize = size(bottleTypes());
        NumberOfBottles = arraySize(1); %how many do we want to simulate


    end
