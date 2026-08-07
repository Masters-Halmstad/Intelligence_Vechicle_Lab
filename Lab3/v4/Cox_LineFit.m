%
% Fits (by translation and rotation) data points to a set of 
% line segments, i.e. applying the Cox's algorithm
% function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose)
% 
function [ddx,ddy,dda,C] = Cox_LineFit(ANG, DIS, POSE, LINEMODEL, SensorPose)
    % Init variables
    ddx = 0; ddy = 0; dda = 0;
    Rx = POSE(1,1); Ry = POSE(2,1); Ra = POSE(3,1); 
    sALFA = SensorPose(1); sBETA = SensorPose(2); sGAMMA = SensorPose(3);
    max_iterations = 1; % <------YOU NEED TO CHANGE THIS NUMBER
    no_update = 0;
    
    % Step 0 - Normal vectors (length = 1) to the line segments
    % -> Add your code here
    
    % The Loop - REPEAT UNTIL THE PROCESS CONVERGE
    for iteration = 1:max_iterations,
        % Step 1 Translate and rotate data points
            % 1.1) Relative measurements => Sensor co-ordinates
            %-> Add your code here

            % 1.2) Sensor co-ordinates => Robot co-ordinates
            %-> Add your code here

            % 1.3) Robot co-ordinates => World co-ordinates
            %-> Add your code here
      
        % Step 2 Find targets for data points
        %-> Add your code here
        
        % Step 3 Set up linear equation system, find b = (dx,dy,da)' from the LS
        %-> Add your code here
        
        b = POSE/max_iterations; % <--- You shall change this! This is only
        % for demonstation, i.e. return the same pose as sent in.
        C = 0; % Covarince matrix
        
        % Step 4 Add latest contribution to the overall congruence 
        ddx = ddx + b(1);
        ddy = ddy + b(2);
        dda = dda + b(3);
        
        % Step 5  Check if the process has converged
        %-> Add your code here
    end;

    