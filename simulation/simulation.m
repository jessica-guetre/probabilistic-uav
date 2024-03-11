close all;

% --------------------------- CONSTANTS -----------------------------------
simTime = 1000;
elevationLimits = [-90 -80];
uavElevation = 20;
gridSize = [100, 100];
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.7; 100, 1.9; 150, 1.5; 200, 1.3], 'Water', [-1, 0.5; 0, 5.5; 50, 3.5; 100, 3.0; 150, 2.4; 200, 2.1], 'Forest', [-1, 1.5], 'Elevation', [-1, 2.0]);
terrainNum = 10;

% --------------------------- ITERATION -----------------------------------
numEpochs = 50; % number of epochs for the training process
numIterations = 500;
learningRate = 0.05; % learning rate for weight adjustment
weightVals = unique(cat(2, linspace(0, 1, numEpochs - round(numEpochs/2)), linspace(1, 5, round(numEpochs/2))));
featureMapFigure = false;
successDistanceFigure = false;

simulationData = struct;
simulationData.bestPerformance = Inf; % Keep track of the best performance
simulationData.bestWeight = 1;
simulationData.epochs(1).weight = 1;
weightUpdates = rand;

filename = 'simulation2.mat';

% --------------------------- TARGET AND UAV POSITIONS --------------------
targetRois = zeros(terrainNum, numIterations, 6);
for terrain = 1:terrainNum
    [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, false);
    for iter = 1:numIterations
        targetRois(terrain, iter, :) = getTarget(probabilityGrid); % get numIterations total target locations
    end
end

uavInitialPositions = [randi(gridSize(2), numIterations, 1), randi(gridSize(1), numIterations, 1), repmat(uavElevation, numIterations, 1)];

% --------------------------- EPOCHS --------------------------------------
for epoch = 1:numEpochs
    weight = weightVals(epoch);
    % weight = simulationData.epochs(epoch).weight;
    allNumWaypoints = zeros(terrainNum, numIterations);

    for terrain = 1:terrainNum
        [featureVertices, probabilityGrid] = getScene(terrain, gridSize, featureMapFigure, squeeze(targetRois(terrain, iter, :)));
        for iter = 1:numIterations
            allNumWaypoints(terrain, iter) = getWaypoints(gridSize, probabilityGrid, uavInitialPositions(iter, :), elevationLimits, weight, squeeze(targetRois(terrain, iter, :)), successDistanceFigure);
        end
    end

    % if epoch < numEpochs
    %     if epoch > 1
    %         if avgWaypoints > simulationData.bestPerformance
    %             learningRate = -learningRate;
    %         end
    %         % weightUpdate = avgWaypoints - simulationData.bestPerformance;
    %     end
    %     weightUpdates = rand;
    %     simulationData.epochs(epoch + 1).weight = simulationData.bestWeight + learningRate * weightUpdates;
    % end

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