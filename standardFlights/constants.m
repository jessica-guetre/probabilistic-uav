% Scenario constants.
updateRate = 3; % in Hz
simTime = 60;
xlimitsScene = [0 100];
ylimitsScene = [0 100];
zlimitsScene = [-5 40];

% UAV platform constants
maxVelocity = 15;
initialVelocity = [15 0 0]; % For spiral and parallel line search
% initialVelocity = [sqrt(maxVelocity^(2)/2) sqrt(maxVelocity^(2)/2) 0]; % For bounce search
% initialPosition = [mean(xlimitsScene) mean(ylimitsScene) 20]; % For spiral search
initialPosition = [10 10 20];
% initialPosition = [80 80 20]; % For testing

% Spiral search constants.
maxSpeed = 15; % Maximum speed
segmentLength = 20; % Initial length of each spiral segment
segmentStep = 20; % Amount to increase the spiral length after each full cycle

% Parallel line search constants.
rowHeight = 20;

% Lidar constants
maxRange = 30;
azimuthResolution = 0.6;
elevationResolution = 2.5;
elevationLimits = [-90 -80];