% --------------------------- CONSTANTS ---------------------------
initialPosition = [10 10 20];
updateRate = 3; % in Hz
simTime = 1000;
elevationLimits = [-90 -80];
uavElevation = initialPosition(3);
gridSize = [100, 100];
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.7; 100, 1.9; 150, 1.5; 200, 1.3], 'Water', [-1, 0.5; 0, 5.5; 50, 3.5; 100, 3.0; 150, 2.4; 200, 2.1], 'Forest', [-1, 1.5], 'Elevation', [-1, 2.0]);

% --------------------------- ITERATION ---------------------------
terrainTypes = {'Type1', 'Type2', 'Type3'};
numIterations = 5;
numEpochs = 1;
weightVals = rand(1, 10) * 5;
learningRate = -0.1;
bestSim = [Inf weightVals(1)]; % time, weights
targetPositions = [20, 20; 80, 60; 30, 50; 60, 40; 50, 20; 20, 40; 60, 60; 20, 70; 70, 60; 70, 70; 20, 80; 30, 60; 80, 40; 20, 20; 80, 50; 30, 70; 60, 80; 30, 80; 80, 70; 60, 50; 50, 40; 70, 30; 50, 80; 20, 60; 40, 40; 30, 20; 60, 70; 80, 30; 40, 20; 50, 70; 40, 80; 80, 80; 70, 80; 70, 50; 40, 30; 30, 40; 40, 50; 40, 60; 20, 30; 50, 30; 70, 20; 80, 20; 50, 60; 70, 40; 30, 30; 60, 20; 20, 50; 50, 50; 40, 70];

% --------------------------- EPOCHS ---------------------------
for epoch = 1:numEpochs
    allWaypointIndices = [];

    for tType = terrainTypes
        featureVertices = getFeatureVertices(tType{1}, gridSize);
        probabilityGrid = terrainProbabilities(featureVertices, gridSize, probabilities);
        figure(1); % For Feature Probability Heatmap
        imagesc(probabilityGrid);
        colorbar;
        axis equal;
        title('Feature Probability Heatmap');
        set(gca, 'YDir', 'normal');
        xlim([1 gridSize(1)]);
        ylim([1 gridSize(2)]);
        xlabel('x (m)');
        ylabel('y (m)');

        for iter = 1:numIterations
            waypointIndex = getWaypoints(tType{1}, gridSize, probabilityGrid, initialPosition, elevationLimits, uavElevation, weightVals(epoch), targetPositions(iter,:));
            fprintf('Terrain %s, WayPointIndices %d, Target Position %d %d\n', tType{1}, waypointIndex, targetPositions(iter,1), targetPositions(iter,2));
            allWaypointIndices = [allWaypointIndices; waypointIndex];
        end
    end

    % weightUpdates = rand(1, 2);
    % oldWeights = weights;
    % 
    % if avgSimTime < bestSim(1)
    %     bestSim = [avgSimTime weights];
    % else
    %     learningRate = learningRate * -1; % Change sign of learning rate if worse
    %     weights = bestSim(2:3); % Use best weights as starting point
    % end
    % 
    % for i = 1:length(weights)
    %     weights(i) = weights(i) + learningRate * weightUpdates(i);
    % end

    fprintf('Epoch: %d, AvgWaypointIndices: %.5f, Weights: %.5f\n', epoch, mean(allWaypointIndices), weightVals(epoch).');
    % fprintf('Epoch: %d, AvgSimTime: %.5f, BestSimTime: %.5f, New Weights: %.5f %.5f, Old Weights: %.5f %.5f, Weight Updates: %.5f %.5f\n', epoch, avgSimTime, bestSim(1), weights.', oldWeights.', weightUpdates.');
end

% --------------------------- WAYPOINTS ---------------------------
function waypointIndex = getWaypoints(selectedTerrainType, gridSize, probabilityGrid, initialPosition, elevationLimits, uavElevation, weight, targetPosition)
    maxNumWaypoints = gridSize(1) * gridSize(2) / 2;
    waypoints = zeros(1, 3, maxNumWaypoints);
    orientation = zeros(1, 3, maxNumWaypoints);
    currentPosition = initialPosition;
    visited = zeros(gridSize);
    visited(currentPosition(1), currentPosition(2)) = 1;
    waypointIndex = 1;
    waypoints(1, :, waypointIndex) = currentPosition;
    orientation(1, :, waypointIndex) = [0, 0, 0];

    targetPositions = [30, 30];
    targetVertices = [targetPosition; targetPosition + [2, 0]; targetPosition + [2, 2]; targetPosition + [0, 2]];
    featureVertices = getFeatureVertices(selectedTerrainType, gridSize);

    figure(2); % Feature Map
    hold on;
    axis equal;
    title('Feature Map');
    xlim([1 gridSize(1)]);
    ylim([1 gridSize(2)]);
    xlabel('x (m)');
    ylabel('y (m)');
    title('Map of Features');
    colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667], 'Field', [0.4667 0.6745 0.1882], 'Target', [1 0 0]);
    patch('Vertices', [1, 1; gridSize(1), 1; gridSize(1), gridSize(2); 1, gridSize(2)], 'Faces', [1, 2, 3, 4], 'FaceColor', colors.('Field'), 'EdgeColor', 'none');
    legendEntries = {'Field'};
    colorsForLegend = colors.('Field');
    for featureType = fieldnames(featureVertices)'
        featureName = featureType{1};
        featureArray = featureVertices.(featureName);
        color = colors.(featureName);
        for i = 1:length(featureArray)
            vertices = featureArray{i};
            patch('Vertices', vertices, 'Faces', 1:size(vertices, 1), 'FaceColor', color, 'EdgeColor', 'none');
        end
        legendEntries{end+1} = featureName;
        colorsForLegend(end+1, :) = color;
    end
    patch('Vertices', targetVertices, 'Faces', [1, 2, 3, 4], 'FaceColor', colors.Target, 'EdgeColor', 'none');
    legendEntries{end + 1} = 'Target';
    colorsForLegend(end + 1, :) = colors.Target;
    for i = 1:length(legendEntries)
        h(i) = patch(NaN, NaN, colorsForLegend(i,:), 'EdgeColor', 'none');
    end
    legend(h, legendEntries);
    hold off;

    % figure(3); % For Success Grid Heatmap
    % figure(4); % For Proximity Grid Heatmap

    targetFound = false;
    while any(visited(:) == 0)
        distanceGrid = zeros(gridSize);
        successGrid = zeros(gridSize);
    
        for x = 1:gridSize(1)
            for y = 1:gridSize(2)
                distanceGrid(x, y) = sqrt((currentPosition(1) - x)^2 + (currentPosition(2) - y)^2);
            end
        end
    
        distanceGrid = 1 - (distanceGrid - min(distanceGrid(:))) / max(max(distanceGrid(:)) - min(distanceGrid(:)), 0.01); % Normalize and invert
    
        for x = 1:gridSize(1)
            for y = 1:gridSize(2)
                if visited(x, y) == 0
                    successGrid(x, y) = distanceGrid(x, y) * weight + probabilityGrid(x, y);
                end
            end
        end
    
        successGrid = (successGrid - min(successGrid(:))) / max(max(successGrid(:)) - min(successGrid(:)), 0.01);
        maxSuccess = max(successGrid(:));
        [successX, successY] = find(successGrid == maxSuccess, 1, 'first');
        shortestDistance = inf;
        successPosition = [successX(1), successY(1), uavElevation];
    
        for i = 1:length(successX)
            distanceToCell = abs(successX(i) - currentPosition(1)) + abs(successY(i) - currentPosition(2));
            if distanceToCell < shortestDistance
                shortestDistance = distanceToCell;
                successPosition = [successX(i), successY(i), 20];
            elseif distanceToCell == shortestDistance
                break;
            end
        end
    
        if currentPosition(1) <= successPosition(1)
            xPositions = (currentPosition(1)+1):1:successPosition(1);
        else
            xPositions = (currentPosition(1)-1):-1:successPosition(1);
        end
        
        if currentPosition(2) <= successPosition(2)
            yPositions = (currentPosition(2)+1):1:successPosition(2);
        else
            yPositions = (currentPosition(2)-1):-1:successPosition(2);
        end
    
        proposedPath = [];
        for x = xPositions
            proposedPath = [proposedPath; x, currentPosition(2), 20];
        end
        for y = yPositions
            proposedPath = [proposedPath; successPosition(1), y, 20];
        end
    
        for i = 1:size(proposedPath, 1)
            waypointIndex = waypointIndex + 1;
            waypoints(1, :, waypointIndex) = proposedPath(i, :);
            if i > 1
                dx = proposedPath(i, 1) - proposedPath(i-1, 1);
                dy = proposedPath(i, 2) - proposedPath(i-1, 2);
                yaw = atan2(dy, dx);
                orientation(1, :, waypointIndex) = [yaw, 0, 0];
            else
                orientation(1, :, waypointIndex) = orientation(1, :, waypointIndex - 1);
            end
        end
        currentPosition = successPosition;
        visited = updateVisitedFromLidar(proposedPath, visited, gridSize, elevationLimits, uavElevation);

        % fprintf('Number of cells not visited yet: %d\n', sum(sum(visited == 0)));
        for x = targetPosition(1):(targetPosition(1) + 2)
            for y = targetPosition(2):(targetPosition(2) + 2)
                % fprintf('x %d y %d visited(x,y) %d\n', x, y, visited(x,y));
                if visited(x, y) == 1
                    targetFound = true;
                end
            end
        end
    
        % % Update Success Grid Heatmap
        % figure(3);
        % imagesc(successGrid);
        % colorbar;
        % axis equal;
        % title('Success Grid Heatmap');
        % set(gca, 'YDir', 'normal');
        % xlabel('x (m)');
        % ylabel('y (m)');
        % drawnow;

        % % Update Proximity Grid Heatmap
        % figure(4);
        % imagesc(distanceGrid);
        % colorbar;
        % axis equal;
        % title('Proximity Grid Heatmap');
        % set(gca, 'YDir', 'normal');
        % xlabel('x (m)');
        % ylabel('y (m)');
        % drawnow; % Force display update
    
        if targetFound == true
            fprintf('Target found within %d waypoints.\n', waypointIndex);
            break;
        end
    end
end

function visited = updateVisitedFromLidar(proposedPath, visited, gridSize, elevationLimits, uavElevation)
    elevationRange = abs(deg2rad(elevationLimits(1) - elevationLimits(2)));
    maxHorizontalRange = tan(elevationRange) * abs(uavElevation);
    for pathIdx = 1:size(proposedPath, 1)
        currentPathPosition = proposedPath(pathIdx, :);
        for x = 1:gridSize(1)
            for y = 1:gridSize(2)
                horizontalDistance = sqrt((currentPathPosition(1) - (x-1))^2 + (currentPathPosition(2) - (y-1))^2);
                if horizontalDistance <= maxHorizontalRange
                    visited(x, y) = 1;
                end
            end
        end
    end
end

function featureVertices = getFeatureVertices(terrainType, gridSize)
    featureVertices = struct();

    scaleFactorX = gridSize(1) / 100;
    scaleFactorY = gridSize(2) / 100;

    switch terrainType
        case 'Type1'
            featureVertices.Trail = {[0, 30*scaleFactorY; gridSize(1), 30*scaleFactorY; gridSize(1), 35*scaleFactorY; 0, 35*scaleFactorY]};
            featureVertices.Water = {
                [40*scaleFactorX, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 30*scaleFactorY; 40*scaleFactorX, 30*scaleFactorY],
                [40*scaleFactorX, 35*scaleFactorY; 45*scaleFactorX, 35*scaleFactorY; 45*scaleFactorX, 90*scaleFactorY; 40*scaleFactorX, 90*scaleFactorY]
            };
            featureVertices.Forest = {
                [20*scaleFactorX, 0; 40*scaleFactorX, 0; 40*scaleFactorX, 10*scaleFactorY; 20*scaleFactorX, 10*scaleFactorY],
                [45*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 10*scaleFactorY; 45*scaleFactorX, 10*scaleFactorY],
                [60*scaleFactorX, 10*scaleFactorY; gridSize(1), 10*scaleFactorY; gridSize(1), 20*scaleFactorY; 60*scaleFactorX, 20*scaleFactorY]
            };
            featureVertices.Elevation = {
                [0, 98*scaleFactorY; gridSize(1), 98*scaleFactorY; gridSize(1), gridSize(2); 0, gridSize(2)],
                [10*scaleFactorX, 90*scaleFactorY; gridSize(1), 90*scaleFactorY; gridSize(1), 98*scaleFactorY; 10*scaleFactorX, 98*scaleFactorY]
            };
        case 'Type2'
            featureVertices.Trail = {[0, 20*scaleFactorY; gridSize(1), 20*scaleFactorY; gridSize(1), 25*scaleFactorY; 0, 25*scaleFactorY]};
            featureVertices.Water = {[0, 50*scaleFactorY; gridSize(1), 50*scaleFactorY; gridSize(1), 60*scaleFactorY; 0, 60*scaleFactorY]};
            featureVertices.Forest = {
                [20*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 20*scaleFactorY; 20*scaleFactorX, 20*scaleFactorY],
                [20*scaleFactorX, 25*scaleFactorY; gridSize(1), 25*scaleFactorY; gridSize(1), 45*scaleFactorY; 20*scaleFactorX, 45*scaleFactorY]
            };
            featureVertices.Elevation = {[10*scaleFactorX, 90*scaleFactorY; gridSize(1), 90*scaleFactorY; gridSize(1), 98*scaleFactorY; 10*scaleFactorX, 98*scaleFactorY]};
        case 'Type3'
            featureVertices.Trail = {[10*scaleFactorX, 0; 20*scaleFactorX, 0; 20*scaleFactorX, gridSize(2); 10*scaleFactorX, gridSize(2)]};
            featureVertices.Water = {
                [50*scaleFactorX, 0; 60*scaleFactorX, 0; 60*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, gridSize(2); 50*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {[60*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 30*scaleFactorY; 60*scaleFactorX, 30*scaleFactorY]};
            featureVertices.Elevation = {[70*scaleFactorX, 70*scaleFactorY; gridSize(1), 70*scaleFactorY; gridSize(1), gridSize(2); 70*scaleFactorX, gridSize(2)]};
        case 'Type4'
            featureVertices.Trail = {[10*scaleFactorX, 0; 15*scaleFactorX, 0; 15*scaleFactorX, gridSize(2); 10*scaleFactorX, gridSize(2)]};
            featureVertices.Water = {
                [30*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, gridSize(2); 30*scaleFactorX, gridSize(2)],
                [30*scaleFactorX, 20*scaleFactorY; gridSize(1), 20*scaleFactorY; gridSize(1), 30*scaleFactorY; 30*scaleFactorX, 30*scaleFactorY]
            };
            featureVertices.Forest = {
                [50*scaleFactorX, 50*scaleFactorY; gridSize(1) - 30*scaleFactorX, 50*scaleFactorY; gridSize(1) - 30*scaleFactorX, gridSize(2) - 20*scaleFactorY; 50*scaleFactorX, gridSize(2) - 20*scaleFactorY],
                [40*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 20*scaleFactorY; 40*scaleFactorX, 20*scaleFactorY],
                [0, gridSize(2) - 20*scaleFactorY; 10*scaleFactorX, gridSize(2) - 20*scaleFactorY; 10*scaleFactorX, gridSize(2); 0, gridSize(2)]
            };
            featureVertices.Elevation = {[55*scaleFactorX, gridSize(2) - 20*scaleFactorY; 85*scaleFactorX, gridSize(2) - 20*scaleFactorY; 85*scaleFactorX, gridSize(2) - 5*scaleFactorY; 55*scaleFactorX, gridSize(2) - 5*scaleFactorY]};
        case 'Type5'
            featureVertices.Trail = {
                [0, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; 0, gridSize(2) - 35*scaleFactorY],
                [gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2); gridSize(1) - 40*scaleFactorX, gridSize(2)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(1) - 10*scaleFactorX, 15*scaleFactorY; gridSize(1), 15*scaleFactorY; gridSize(1), gridSize(2); gridSize(1) - 10*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 10*scaleFactorY; gridSize(1) - 40*scaleFactorX, 10*scaleFactorY]};
    end
end

function probabilityGrid = terrainProbabilities(featureVertices, gridSize, probabilities)
    probabilityGrid = ones(gridSize);

    % Set the probability for the cell containing the feature itself
    for featureType = fieldnames(featureVertices)'
        probabilityInfo = probabilities.(featureType{1});
        binaryMask = zeros(gridSize);
        for j = 1:length(featureVertices.(featureType{1}))
            vertices = featureVertices.(featureType{1}){j};
            mask = poly2mask(vertices(:,1), vertices(:,2), gridSize(1), gridSize(2));
            binaryMask(mask) = 1;
        end
        probabilityGrid(binaryMask == 1) = probabilityInfo(1, 2);
    end

    % Set the probabilities for the cells around the feature
    for featureType = fieldnames(featureVertices)'
        probabilityInfo = probabilities.(featureType{1});

        binaryMask = zeros(gridSize);
        for j = 1:length(featureVertices.(featureType{1}))
            vertices = featureVertices.(featureType{1}){j};
            mask = poly2mask(vertices(:,1), vertices(:,2), gridSize(1), gridSize(2));
            binaryMask(mask) = 1;
        end

        distanceTransform = bwdist(binaryMask);

        for k = 2:size(probabilityInfo, 1) - 1
            lowerBoundDistance = probabilityInfo(k, 1);
            upperBoundDistance = probabilityInfo(k+1, 1);
            lowerBoundProbability = probabilityInfo(k, 2);
            upperBoundProbability = probabilityInfo(k+1, 2);
            indices = distanceTransform >= lowerBoundDistance & distanceTransform < upperBoundDistance;
            
            slope = (upperBoundProbability - lowerBoundProbability) / (upperBoundDistance - lowerBoundDistance);
            probabilityGrid(indices) = probabilityGrid(indices) + (lowerBoundProbability + slope * (distanceTransform(indices) - lowerBoundDistance));
        end
    end

    probabilityGrid = (probabilityGrid - min(probabilityGrid(:))) / (max(probabilityGrid(:)) - min(probabilityGrid(:)));
end