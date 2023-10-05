% Connor Keogh      13220482
% Nicholas Mullan   11111111
% Patrick Hore      11111111

%% Set up Environment











%% Set up bottle initial locations
% note no orientation considering bottle is mirrored about Z-axis. 

%% Objects
%Set the initial position for each Object
NumberOfObjects = 9; 

InitialObjectLocationsArray = [
    %Location XYZ   
    -0.32, 0.45, 0.5;           %Object 1
    -0.163, 0.45, 0.5;          %Object 2
    -0.026, 0.45, 0.5;          %Object 3
    -0.32, 0.45, 0.54;          %Object 4
    -0.163, 0.45, 0.54;         %Object 5
    -0.026, 0.45, 0.54;         %Object 6
    -0.32, 0.45, 0.58;          %Object 7
    -0.163, 0.45, 0.58;         %Object 8
    -0.026, 0.45, 0.58;         %Object 9
];

IntermediateObjectLocationsArray = [
    0, 0.45, 0;                 %Object 1
    0, 0.45, 0;                 %Object 2
    0, 0.45, 0;                 %Object 3
    0, 0.45, 0;                 %Object 4
    0, 0.45, 0;                 %Object 5
    0, 0.45, 0;                 %Object 6
    0, 0.45, 0;                 %Object 7
    0, 0.45, 0;                 %Object 8
    0, 0.45, 0;                 %Object 9

]

%The aim is to arrange the Objects end-to-end, forming 3 rows, with each
%row accommodating 3 Objects. This arrangement culminates in a stacked configuration that adheres to these
%specifications
FinalObjectLocationsArray = [
    %Location XYZ   Rotation
    -0.3, -0.45, 0.5;           %Object 1
    -0.163, -0.45, 0.5;         %Object 2
    -0.026, -0.45, 0.5;         %Object 3
    -0.3, -0.45, 0.54;          %Object 4
    -0.163, -0.45, 0.54;        %Object 5
    -0.026, -0.45, 0.54;        %Object 6
    -0.3, -0.45, 0.58;          %Object 7
    -0.163, -0.45, 0.58;        %Object 8
    -0.026, -0.45, 0.58;        %Object 9
];

%% Move objects
function CollectOrderFromCustomer(drinkChoice, customerPose)
    % Robot A collects bottle and passes it to an intermediate position for
    % robot B to pick up and give to the customers position


    Steps = 50;
    PoseGuess = [-0.4, deg2rad(90), deg2rad(-45), deg2rad(-90), deg2rad(45),deg2rad(90), 0];





    % Use Robot A to move bottle from initial pose to intermediate pose


    % Use Robot B to move bottle from intermediate pose to final pose



end

%% collisiondetection









