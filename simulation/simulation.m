close all;

% --------------------------- CONSTANTS -----------------------------------
uavElevation = 12;
gridSize = [100, 100];
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.7; 100, 1.9; 150, 1.5; 200, 1.3], 'Water', [-1, 0.5; 0, 5.5; 50, 3.5; 100, 3.0; 150, 2.4; 200, 2.1], 'Forest', [-1, 1.5], 'Elevation', [-1, 2.0]);
numTerrains = 10; % 10;
numIterations = 50; % 500;
featureMapFigure = false;
successDistanceFigure = false;
targetFactors = [0, 0.2, 0.4, 0.6, 0.8, 1, 1.5, 2.0, 2.5, 3, 4, 5, 10];

% --------------------------- TARGET POSITIONS ----------------------------
targetRois = zeros(numTerrains, numIterations, length(targetFactors), 6);
for f = 1:length(targetFactors)
    for t = 1:numTerrains
        [featureVertices, probabilityGrid] = getScene(t, gridSize, false);
        for i = 1:numIterations
            targetRois(t, i, f, :) = getTarget(probabilityGrid, targetFactors(f)); % get numIterations total target locations
        end
    end
end

% --------------------------- PARALLEL ------------------------------------
flightType = 'parallelLine';
uavInitialPositions = [repmat(2, numIterations, 1), repmat(10, numIterations, 1), repmat(uavElevation, numIterations, 1)];
parallelLineData = struct;
avgWaypointsParallel = zeros(1, length(targetFactors));
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
    fprintf('Target Factor: %.1f, Avg Waypoints: %.2f\n', targetFactors(f), avgWaypoints);

    avgWaypointsParallel(f) = avgWaypoints;
    parallelLineData.targetFactor(f).avgWaypoints = avgWaypoints;
    parallelLineData.targetFactor(f).allNumWaypoints = allNumWaypoints;
end

save('parallelLine.mat', 'parallelLineData');

% --------------------------- SPIRAL --------------------------------------
flightType = 'spiral';
uavInitialPositions = [repmat(round(gridSize(2)/2), numIterations, 1), repmat(round(gridSize(1)/2), numIterations, 1), repmat(uavElevation, numIterations, 1)];
spiralData = struct;
avgWaypointsSpiral = zeros(1, length(targetFactors));
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
    fprintf('Target Factor: %.1f, Avg Waypoints: %.2f\n', targetFactors(f), avgWaypoints);

    avgWaypointsSpiral(f) = avgWaypoints;
    spiralData.targetFactor(f).avgWaypoints = avgWaypoints;
    spiralData.targetFactor(f).allNumWaypoints = allNumWaypoints;
end

save('spiral.mat', 'spiralData');

% --------------------------- PROBABILISTIC -------------------------------
flightType = 'probabilistic';
uavInitialPositions = [repmat(round(gridSize(2)/2), numIterations, 1), repmat(round(gridSize(1)/2), numIterations, 1), repmat(uavElevation, numIterations, 1)];
probabilisticData = struct;
distanceFactors = [0, 0.2, 0.4, 0.6, 0.8, 1, 1.5, 2.0, 2.5, 3, 4, 5, 10];
avgWaypointsProbabilistic = zeros(length(distanceFactors), length(targetFactors));

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
                fprintf('Target Factor: %.1f, Distance Factor: %.1f, Waypoints: %.2f\n', targetFactors(f), distanceFactors(w), allNumWaypoints(t, i));
            end
        end

        avgWaypointsProbabilistic(w, f) = avgWaypoints;
        avgWaypoints = mean(allNumWaypoints(:)); % flatten array to calculate average
        if avgWaypoints < probabilisticData.targetFactor(f).bestPerformance
            probabilisticData.targetFactor(f).bestPerformance = avgWaypoints;
            probabilisticData.targetFactor(f).bestWeight = distanceFactors(w);
        end

        probabilisticData.targetFactor(f).distanceFactor(w).distanceFactor = distanceFactors(w); % store data for each epoch in the structure
        probabilisticData.targetFactor(f).distanceFactor(w).avgWaypoints = avgWaypoints;
        probabilisticData.targetFactor(f).distanceFactor(w).allNumWaypoints = allNumWaypoints;

        fprintf('Target Factor: %.1f, Distance Factor: %.1f, Avg Waypoints: %.2f\n', targetFactors(f), distanceFactors(w), avgWaypoints);
    end
end

save('probabilistic.mat', 'probabilisticData');

% --------------------------- ALL DATA ------------------------------------
allAvgWaypoints = [avgWaypointsParallel; avgWaypointsSpiral; avgWaypointsProbabilistic];
avgWaypointsTable = array2table(allAvgWaypoints);
disp(avgWaypointsTable);
save('avgWaypointsTable.mat', 'avgWaypointsTable');