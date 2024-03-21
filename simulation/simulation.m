close all;

% --------------------------- CONSTANTS -----------------------------------
uavElevation = 12;
gridSize = [100, 100];
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.7; 100, 1.9; 150, 1.5; 200, 1.3], 'Water', [-1, 0.5; 0, 5.5; 50, 3.5; 100, 3.0; 150, 2.4; 200, 2.1], 'Forest', [-1, 1.5], 'Elevation', [-1, 2.0]);
numTerrains = 10; % 10;
numIterations = 100; % 500;
featureMapFigure = false;
successDistanceFigure = false;
targetFactors = [0.1, 0.5, 0.0, 1.0, 2.0, 3.0, 4.0, 5.0];

% --------------------------- TARGET AND UAV POSITIONS --------------------
targetRois = zeros(numTerrains, numIterations, length(targetFactors), 6);
for f = 1:length(targetFactors)
    for t = 1:numTerrains
        [featureVertices, probabilityGrid] = getScene(numTerrains, gridSize, false);
        for i = 1:numIterations
            targetRois(t, i, f, :) = getTarget(probabilityGrid, targetFactors(f)); % get numIterations total target locations
        end
    end
end

% --------------------------- PROBABILISTIC -------------------------------
flightType = 'probabilistic';
% uavInitialPositions = [repmat(10, numIterations, 1), repmat(10, numIterations, 1), repmat(uavElevation, numIterations, 1)];
uavInitialPositions = [repmat(round(gridSize(2)/2), numIterations, 1), repmat(round(gridSize(1)/2), numIterations, 1), repmat(uavElevation, numIterations, 1)];
% distanceFactors = unique(cat(2, linspace(0.5, 1, numEpochs - round(numEpochs/2)), linspace(1, 10, round(numEpochs/2) + 1)));
distanceFactors = [0, 0.2, 0.4, 0.6, 0.8, 1, 1.5, 2.0, 2.5, 3, 4, 5, 10];

probabilisticData = struct;

fprintf('\nProbabilistic\n');

for f = 1:length(targetFactors)
    probabilisticData.targetFactor(f).bestPerformance = Inf; % Keep track of the best performance
    probabilisticData.targetFactor(f).bestWeight = 1;

    for w = 1:length(distanceFactors)
        allNumWaypoints = zeros(numTerrains, numIterations);

        for t = 1:numTerrains
            for i = 1:numIterations
                [featureVertices, probabilityGrid] = getScene(t, gridSize, featureMapFigure, squeeze(targetRois(t, i, f, :)));
                allNumWaypoints(t, i) = getWaypoints(flightType, gridSize, probabilityGrid, uavInitialPositions(i, :), distanceFactors(w), squeeze(targetRois(t, i, f, :)), successDistanceFigure);
            end
        end

        avgWaypoints = mean(allNumWaypoints(:)); % flatten array to calculate average
        if avgWaypoints < probabilisticData.targetFactor(f).bestPerformance
            probabilisticData.targetFactor(f).bestPerformance = avgWaypoints;
            probabilisticData.targetFactor(f).bestWeight = distanceFactors(w);
        end

        probabilisticData.targetFactor(f).distanceFactor(w).distanceFactor = distanceFactors(w); % store data for each epoch in the structure
        probabilisticData.targetFactor(f).distanceFactor(w).avgWaypoints = avgWaypoints;
        probabilisticData.targetFactor(f).distanceFactor(w).allNumWaypoints = allNumWaypoints;

        fprintf('Target Factor: %.1f, Weight: %.1f, Avg Waypoints: %.2f\n', targetFactors(f), distanceFactors(w), avgWaypoints);
    end
end

% Save the accumulated data at the end of all epochs
save('probabilistic.mat', 'probabilisticData');

% --------------------------- PARALLEL ------------------------------------
flightType = 'parallelLine';
uavInitialPositions = [repmat(10, numIterations, 1), repmat(10, numIterations, 1), repmat(uavElevation, numIterations, 1)];
parallelLineData = struct;
allNumWaypoints = zeros(numTerrains, numIterations);

fprintf('\nParallel\n');

for f = 1:length(targetFactors)
    for t = 1:numTerrains
        for i = 1:numIterations
            [featureVertices, probabilityGrid] = getScene(t, gridSize, featureMapFigure, squeeze(targetRois(t, i, f, :)));
            allNumWaypoints(t, i) = getWaypoints(flightType, gridSize, probabilityGrid, uavInitialPositions(i, :), 1, squeeze(targetRois(t, i, f, :)), successDistanceFigure);
        end
    end
    avgWaypoints = mean(allNumWaypoints(:)); % flatten array to calculate average
    fprintf('Target Factor: %d, Avg Waypoints: %.2f\n', targetFactors(f), avgWaypoints);

    parallelLineData.targetFactor(f).avgWaypoints = avgWaypoints;
    parallelLineData.targetFactor(f).allNumWaypoints = allNumWaypoints;
end

save('parallelLine.mat', 'parallelLineData');

% --------------------------- SPIRAL --------------------------------------
flightType = 'spiral';
uavInitialPositions = [repmat(round(gridSize(2)/2), numIterations, 1), repmat(round(gridSize(1)/2), numIterations, 1), repmat(uavElevation, numIterations, 1)];
spiralData = struct;
allNumWaypoints = zeros(numTerrains, numIterations);

fprintf('\nSpiral\n');

for f = 1:length(targetFactors)
    for t = 1:numTerrains
        for i = 1:numIterations
            [featureVertices, probabilityGrid] = getScene(t, gridSize, featureMapFigure, squeeze(targetRois(t, i, f, :)));
            allNumWaypoints(t, i) = getWaypoints(flightType, gridSize, probabilityGrid, uavInitialPositions(i, :), 1, squeeze(targetRois(t, i, f, :)), successDistanceFigure);
        end
    end
    avgWaypoints = mean(allNumWaypoints(:)); % flatten array to calculate average
    fprintf('Target Factor: %d, Avg Waypoints: %.2f\n', targetFactors(f), avgWaypoints);

    spiralData.targetFactor(f).avgWaypoints = avgWaypoints;
    spiralData.targetFactor(f).allNumWaypoints = allNumWaypoints;
end

save('spiral.mat', 'spiralData');
