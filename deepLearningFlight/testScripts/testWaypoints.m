initialPosition = [10 10 20];
elevationLimits = [-90 -80];
updateRate = 1;
simTime = 60;
gridSize = [100, 100];
selectedTerrainType = 'Type1';
targetPosition = [50, 50];
featureVertices = getFeatureVertices(selectedTerrainType, gridSize);
weight = 1;
uavElevation = initialPosition(3);
[gridScene, probabilityGrid, roi] = getScene(updateRate, simTime, gridSize, selectedTerrainType, targetPosition);
targetPosition = [roi(1), roi(3)];
targetVertices = [targetPosition; targetPosition + [1, 0]; targetPosition + [1, 1]; targetPosition + [0, 1]];

maxNumWaypoints = gridSize(1) * gridSize(2) / 2;
waypoints = zeros(1, 3, maxNumWaypoints);
orientation = zeros(1, 3, maxNumWaypoints);
currentPosition = [initialPosition(1), initialPosition(2), uavElevation];
visited = zeros(gridSize);
visited(currentPosition(1), currentPosition(2)) = 1;
waypointIndex = 1;
waypoints(1, :, waypointIndex) = currentPosition;
orientation(1, :, waypointIndex) = [0, 0, 0];
maxDistance = sqrt((gridSize(1)-1)^2 + (gridSize(2)-1)^2);

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
figure(2); % For Success Grid Heatmap
figure(3); % For Proximity Grid Heatmap
figure(4); % Feature Map
hold on;
axis equal;
title('Feature Map');
xlim([1 gridSize(1)]);
ylim([1 gridSize(2)]);
xlabel('x (m)');
ylabel('y (m)');
title('Map of Features');
colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667], 'Field', [0.4667 0.6745 0.1882], 'Target', [1 0 0]);
% FIELD
patch('Vertices', [1, 1; gridSize(1), 1; gridSize(1), gridSize(2); 1, gridSize(2)], 'Faces', [1, 2, 3, 4], 'FaceColor', colors.('Field'), 'EdgeColor', 'none');
legendEntries = {'Field'};
colorsForLegend = colors.('Field');
% TARGET
patch('Vertices', targetVertices, 'Faces', 1:size(targetVertices, 1), 'FaceColor', colors.('Target'), 'EdgeColor', 'none');
legendEntries{end + 1} = 'Target';
colorsForLegend(end + 1, :) = colors.Target;
% FEATURES
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
% LEGEND
for i = 1:length(legendEntries)
    h(i) = patch(NaN, NaN, colorsForLegend(i,:), 'EdgeColor', 'none');
end
legend(h, legendEntries);
hold off;

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

    for x = roi(1):roi(2)
        for y = roi(3):roi(4)
            if visited(x, y) == 1
                targetFound = true;
            end
        end
    end

    % Update Success Grid Heatmap
    figure(2);
    imagesc(successGrid);
    colorbar;
    axis equal;
    title('Success Grid Heatmap');
    set(gca, 'YDir', 'normal');
    xlabel('x (m)');
    ylabel('y (m)');
    drawnow; % Force display update

    % Update Proximity Grid Heatmap
    figure(3);
    imagesc(distanceGrid);
    colorbar;
    axis equal;
    title('Proximity Grid Heatmap');
    set(gca, 'YDir', 'normal');
    xlabel('x (m)');
    ylabel('y (m)');
    drawnow; % Force display update

    if targetFound
        fprintf('Target found within %d waypoints.\n', waypointIndex);
        break;
    end
end

waypoints = waypoints(:,:,1:waypointIndex);
orientation = orientation(:,:,1:waypointIndex);

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