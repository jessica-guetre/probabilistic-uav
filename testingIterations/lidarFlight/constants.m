% Scenario constants.
updateRate = 10; % in Hz
simTime = 15;
xlimitsScene = [-5 35];
ylimitsScene = [-5 35];
zlimitsScene = [-4 0];

% Target constants.
xboundsTarget = [20 25];
yboundsTarget = [20 25];
zlimitsTarget = [-4 4];
sizeTarget = 2;

% UAV platform constants
velocity = [4 4 0];
initialPosition = [0 0 5];

% Lidar constants.
maxRange = 20;
azimuthResolution = 0.6;
elevationResolution = 2.5;
elevationLimits = [-90 20];
