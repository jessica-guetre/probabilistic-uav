% Convert weightedGrid to a 3D matrix with a single channel
terrainInput = reshape(weightedGrid, [size(weightedGrid,1), size(weightedGrid,2), 1]);

% Normalize initialPosition (ensure normalization is consistent with training data)
positionInput = initialPosition(1:2); % Assuming normalization is done elsewhere

% Assuming simulationTime is calculated and waypointsVector is prepared as shown previously
targetData = struct('waypoints', waypointsVector, 'simulationTime', simulationTime);

% Before the simulation loop, load your trained network
% Assuming 'trainedNet' is your loaded network

% In place of the manual waypoint generation:
if targetFound % Assuming you set this flag when the target is detected
    % Prepare network input
    % Note: Make sure the inputs are formatted exactly as the network expects
    netInput = {reshape(terrainInput, [size(weightedGrid,1), size(weightedGrid,2), 1]), positionInput};

    % Predict waypoints
    predictedWaypoints = predict(trainedNet, netInput);
    
    % Convert predicted waypoints to the required format
    % This step depends on how you've formatted the output in your network
    % For example, if your output is a flattened vector of (X, Y, Z) coordinates:
    predictedWaypoints = reshape(predictedWaypoints, [3, numel(predictedWaypoints)/3])';
    
    % Use predicted waypoints in your simulation
    % Here you would replace your manual waypoint setting with the use of predicted waypoints
end

% Log data for training
flightData.positions = [flightData.positions; currentPosition];
flightData.terrainTypes = [flightData.terrainTypes; currentTerrainType]; % Adjust as needed
flightData.waypoints = [flightData.waypoints; waypoints];
% Ensure to log simulationTime when the target is found

