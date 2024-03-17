close all;

% --------------------------- CONSTANTS -----------------------------------
simTime = 1000;
elevationLimits = [-90 -80];
uavElevation = 20;
gridSize = [100, 100];
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.7; 100, 1.9; 150, 1.5; 200, 1.3], 'Water', [-1, 0.5; 0, 5.5; 50, 3.5; 100, 3.0; 150, 2.4; 200, 2.1], 'Forest', [-1, 1.5], 'Elevation', [-1, 2.0]);
terrainNum = 1;
flightType = 'spiral'; % 'probabilistic', 'spiral', 'parallelLine'

% --------------------------- ITERATION -----------------------------------
numEpochs = 10; % number of epochs for the training process
numIterations = 200;
learningRate = 0.05; % learning rate for weight adjustment
weightVals = unique(cat(2, linspace(0, 1, numEpochs - round(numEpochs/2)), linspace(1, 5, round(numEpochs/2) + 1)));
featureMapFigure = false;
successDistanceFigure = false;

simulationData = struct;
simulationData.bestPerformance = Inf; % Keep track of the best performance
simulationData.bestWeight = 1;
simulationData.epochs(1).weight = 1;
weightUpdates = rand;

filename = 'simulationNew.mat';

% --------------------------- TARGET AND UAV POSITIONS --------------------
targetRois = zeros(terrainNum, numIterations, 6);
for terrain = 1:terrainNum
    [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, false);
    for iter = 1:numIterations
        targetRois(terrain, iter, :) = getTarget(probabilityGrid); % get numIterations total target locations
    end
end

if strcmp(flightType,'probabilistic')
    uavInitialPositions = [randi(gridSize(2), numIterations, 1), randi(gridSize(1), numIterations, 1), repmat(uavElevation, numIterations, 1)];
else
    uavInitialPositions = [repmat(round(gridSize(2)/2), numIterations, 1), repmat(round(gridSize(1)/2), numIterations, 1), repmat(uavElevation, numIterations, 1)];
end

uavInitialPositions = [repmat(round(gridSize(2)/2), numIterations, 1), repmat(round(gridSize(1)/2), numIterations, 1), repmat(uavElevation, numIterations, 1)];

% --------------------------- EPOCHS --------------------------------------
for epoch = 1:numEpochs
    weight = weightVals(epoch);
    allNumWaypoints = zeros(terrainNum, numIterations);

    for terrain = 1:terrainNum
        for iter = 1:numIterations
            [featureVertices, probabilityGrid] = getScene(terrain, gridSize, featureMapFigure, squeeze(targetRois(terrain, iter, :)));
            allNumWaypoints(terrain, iter) = getWaypoints(flightType, gridSize, probabilityGrid, uavInitialPositions(iter, :), elevationLimits, weight, squeeze(targetRois(terrain, iter, :)), successDistanceFigure);
        end
    end

    avgWaypoints = mean(allNumWaypoints(:)); % flatten array to calculate average
    if avgWaypoints < simulationData.bestPerformance
        simulationData.bestPerformance = avgWaypoints;
        simulationData.bestWeight = weight;
    end

    simulationData.epochs(epoch).weight = weight; % store data for each epoch in the structure
    simulationData.epochs(epoch).avgWaypoints = avgWaypoints;
    simulationData.epochs(epoch).allNumWaypoints = allNumWaypoints;

    fprintf('Epoch: %d, Avg Waypoints: %.2f, Weight: %.5f, Best Num Waypoints: %.2f, Best Weight: %.5f\n', epoch, avgWaypoints, weight, simulationData.bestPerformance, simulationData.bestWeight);
end

% Save the accumulated data at the end of all epochs
save(filename, 'simulationData');