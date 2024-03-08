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
numIterations = 20;
numEpochs = 20;
learningRate = -0.1;
bestSim = [Inf weightVals(1)]; % time, weights
% weightVals = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0];
weight = 1;

targetRoi = zeros(numIterations, 6);
for i = 1:numIterations
    targetRoi(i, :) = getTarget(probabilityGrid); % get numIterations target locations
end

% --------------------------- EPOCHS ---------------------------
createFigure = false;
for epoch = 1:numEpochs
    allWaypointIndices = [];

    for tType = terrainTypes
        [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure);
        for iter = 1:numIterations
            waypointIndex = getWaypoints(gridSize, probabilityGrid, initialPosition, elevationLimits, weight, targetRoi(iter, :));
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
end