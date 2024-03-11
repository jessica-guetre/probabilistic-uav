% --------------------------- CONSTANTS -----------------------------------
simTime = 1000;
elevationLimits = [-90 -80];
uavElevation = 20;
gridSize = [100, 100];
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.7; 100, 1.9; 150, 1.5; 200, 1.3], 'Water', [-1, 0.5; 0, 5.5; 50, 3.5; 100, 3.0; 150, 2.4; 200, 2.1], 'Forest', [-1, 1.5], 'Elevation', [-1, 2.0]);

% --------------------------- ITERATION -----------------------------------
numEpochs = 100; % number of epochs for the training process
numIterations = 100;
learningRate = 0.1; % learning rate for weight adjustment
weightVals = unique(cat(2, linspace(0, 1, round(numEpochs/2)), linspace(1, 5, round(numEpochs/2)) + 1));
bestWeight = weight; % best weight found
bestPerformance = Inf; % initialize best performance as worst case
% performance = zeros(numEpochs,2); % weight, mean num waypoints

% --------------------------- TARGET AND UAV POSITIONS --------------------
targetRois = zeros(numIterations, 6);
for i = 1:numIterations
    targetRois(i, :) = getTarget(probabilityGrid); % get numIterations total target locations
end

uavInitialPositions = [randi(gridSize(1), numIterations, 1), randi(gridSize(2), numIterations, 1), repmat(uavElevation, numIterations, 1)];

% --------------------------- EPOCHS --------------------------------------
featureMapFigure = false;
successDistanceFigure = false;
for epoch = 1:numEpochs
    weight = weightVals(epoch);
    allNumWaypoints = [];

    for terrain = 1:terrainNum
        [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, featureMapFigure);
        for iter = 1:numIterations
            numWaypoints = getWaypoints(gridSize, probabilityGrid, uavInitialPositions(iter, :), elevationLimits, weight, targetRois(iter, :), successDistanceFigure);
            % fprintf('Terrain %s, WayPointIndices %d, Target Position %d %d\n', tType{1}, waypointIndex, targetPositions(iter,1), targetPositions(iter,2));
            allNumWaypoints = [allNumWaypoints; numWaypoints];
        end
    end

    avgWaypoints = mean(allNumWaypoints);
    if avgWaypoints < bestPerformance
        bestPerformance = avgWaypoints;
        bestWeight = weight;
    end

    fprintf('Epoch: %d, Avg Waypoints: %.2f, Weight: %.5f, Best Num Waypoints: %.2f, Best Weight: %.5f\n', epoch, avgWaypoints, weight, bestPerformance, bestWeight);

    % if epoch > 1 && avgWaypoints > lastAvgWaypoints
    %     learningRate = -learningRate; % reverse direction if worse performance
    % end
    % if epoch < numEpochs
    %     weights(epoch + 1) = weights(epoch) + learningRate; % adjust weight for next epoch
    % end
end