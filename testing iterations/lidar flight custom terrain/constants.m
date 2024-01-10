% Scenario constants.
updateRate = 3; % in Hz
simTime = 200;
xlimitsScene = [0 200];
ylimitsScene = [0 200];
zlimitsScene = [920 980];

% Target constants.
xboundsTarget = [40 160];
yboundsTarget = [40 160];
zlimitsTarget = [940 950];
sizeTarget = 2;

% UAV platform constants
maxVelocity = 15;
initialVelocity = [sqrt(maxVelocity^(2)/2) sqrt(maxVelocity^(2)/2) 0];
initialPosition = [10 10 960];
% initialPosition = [160 160 960];

% Lidar constants
maxRange = 20;
azimuthResolution = 0.6;
elevationResolution = 2.5;
elevationLimits = [-90 20];