% --------------------------- CONSTANTS ---------------------------
initialPosition = [10 10 20];
simTime = 1000;
elevationLimits = [-90 -80];
uavElevation = initialPosition(3);
gridSize = [100, 100];
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.7; 100, 1.9; 150, 1.5; 200, 1.3], 'Water', [-1, 0.5; 0, 5.5; 50, 3.5; 100, 3.0; 150, 2.4; 200, 2.1], 'Forest', [-1, 1.5], 'Elevation', [-1, 2.0]);

% --------------------------- ITERATION ---------------------------
terrainTypes = {'Type1', 'Type2', 'Type3', 'Type4', 'Type5'};
numIterations = 5;
numEpochs = 10;
learningRate = -0.1;
bestSim = [Inf weightVals(1)]; % time, weights
% weightVals = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0];
targetPositions = [20, 20; 80, 60; 30, 50; 60, 40; 50, 20; 20, 40; 60, 60; 20, 70; 70, 60; 70, 70; 20, 80; 30, 60; 80, 40; 20, 20; 80, 50; 30, 70; 60, 80; 30, 80; 80, 70; 60, 50; 50, 40; 70, 30; 50, 80; 20, 60; 40, 40; 30, 20; 60, 70; 80, 30; 40, 20; 50, 70; 40, 80; 80, 80; 70, 80; 70, 50; 40, 30; 30, 40; 40, 50; 40, 60; 20, 30; 50, 30; 70, 20; 80, 20; 50, 60; 70, 40; 30, 30; 60, 20; 20, 50; 50, 50; 40, 70];
weight = 1;

% --------------------------- EPOCHS ---------------------------
createFigure = false;
for epoch = 1:numEpochs
    allWaypointIndices = [];

    for tType = terrainTypes
        [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure);
        for iter = 1:numIterations
            waypointIndex = getWaypoints(gridSize, probabilityGrid, initialPosition, elevationLimits, weight, targetPositions(iter,:));
            % fprintf('Terrain %s, WayPointIndices %d, Target Position %d %d\n', tType{1}, waypointIndex, targetPositions(iter,1), targetPositions(iter,2));
            allWaypointIndices = [allWaypointIndices; waypointIndex];
        end
    end

    avgWaypoints = mean(allWaypointIndices);
    weightUpdate = rand;

    if avgWaypoints < bestSim(1)
        bestSim = [avgWaypoints weight];
    else
        learningRate = learningRate * -1; % Change sign of learning rate if worse
        weight = bestSim(2); % Use best weights as starting point
    end

    weight = weight + learningRate * weightUpdate;

    fprintf('Epoch: %d, Avg Waypoints: %.2f, Weights: %.5f, Best Num Waypoints %.2f, Best Weights %.5f\n', epoch, avgWaypoints, weight, bestSim(1), bestSim(2));
    % fprintf('Epoch: %d, AvgSimTime: %.5f, BestSimTime: %.5f, New Weights: %.5f %.5f, Old Weights: %.5f %.5f, Weight Updates: %.5f %.5f\n', epoch, avgSimTime, bestSim(1), weights.', oldWeights.', weightUpdates.');
end