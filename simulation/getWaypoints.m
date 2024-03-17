function waypointIndex = getWaypoints(flightType, gridSize, probabilityGrid, initialPosition, elevationLimits, weight, targetRoi, createFigure)
    uavElevation = initialPosition(3);
    maxNumWaypoints = gridSize(1) * gridSize(2) / 2;
    waypoints = zeros(1, 3, maxNumWaypoints);
    orientation = zeros(1, 4, maxNumWaypoints);
    currentPosition = initialPosition; % Ensure currentPosition is [y, x]
    visited = zeros(gridSize); % gridSize should be [rows (y), columns (x)]
    waypointIndex = 1;
    targetFound = false;
    lidarRange = 10;

    direction = [1, 0];
    segmentLength = 0;

    if createFigure
        figure(1); % Success Grid Heatmap
        figure(2); % Proximity Grid Heatmap
    end

    visited = updateVisitedFromLidar(currentPosition, visited, gridSize, lidarRange);

    while any(visited(:) == 0) && ~targetFound
        [distanceGrid, successGrid] = calculateGrids(currentPosition, gridSize, probabilityGrid, visited, weight);
        if strcmp(flightType,'probabilistic')
            successPosition = findNextPositionProbabilistic(currentPosition, lidarRange, successGrid);
        else
            [successPosition, direction, segmentLength] = findNextPositionSpiral(currentPosition, lidarRange, direction, segmentLength);
        end

        proposedPath = generatePath(currentPosition, successPosition, uavElevation);

        for i = 1:size(proposedPath, 1)
            waypointIndex = waypointIndex + 1;
            waypoints(1, :, waypointIndex) = proposedPath(i, :);
            orientation(1, :, waypointIndex) = calculateOrientation(proposedPath, i);
            currentPosition = [proposedPath(i, 2), proposedPath(i, 1)];
            visited = updateVisitedFromLidar(currentPosition, visited, gridSize, lidarRange);
    
            if checkTargetFound(visited, targetRoi)
                targetFound = true;
                break; % Exit loop once target is found
            end
        end

        if createFigure
            [distanceGrid, successGrid] = calculateGrids(currentPosition, gridSize, probabilityGrid, visited, weight);
            updateHeatmaps(successGrid, distanceGrid, gridSize);
        end
    end

    waypoints = waypoints(1, :, 1:waypointIndex); % Trim to actual size
    orientation = orientation(1, :, 1:waypointIndex); % Trim to actual size
end

function [distanceGrid, successGrid] = calculateGrids(currentPosition, gridSize, probabilityGrid, visited, weight)
    distanceGrid = zeros(gridSize);
    for y = 1:gridSize(1) % Adjusted to iterate over rows first
        for x = 1:gridSize(2) % Adjusted to iterate over columns second
            % Adjusted calculation to correctly compute distance using y-x indexing
            distanceGrid(y, x) = sqrt((currentPosition(2) - x)^2 + (currentPosition(1) - y)^2);
        end
    end

    distanceGrid = 1 - (distanceGrid - min(distanceGrid(:))) / max(max(distanceGrid(:)) - min(distanceGrid(:)), 0.01); % normalize and invert
    successGrid = zeros(gridSize);
    for y = 1:gridSize(1)
        for x = 1:gridSize(2)
            if visited(y, x) == 0 
                successGrid(y, x) = distanceGrid(y, x) * weight + probabilityGrid(y, x); % Adjusted to use y-x indexing
            end
        end
    end

    successGrid = (successGrid - min(successGrid(:))) / max(max(successGrid(:)) - min(successGrid(:)), 0.01);
end

function successPosition = findNextPositionProbabilistic(currentPosition, lidarRange, successGrid)
    maxSuccess = max(successGrid(:));
    [successY, successX] = find(successGrid == maxSuccess, 20, 'first'); % Note the switch to [successY, successX]
    shortestDistance = inf;

    for i = 1:length(successX)
        ydistanceToCell = successY(i) - currentPosition(1);
        xdistanceToCell = successX(i) - currentPosition(2);
        distanceToCell = sqrt(ydistanceToCell^2 + xdistanceToCell^2);
        if  i == 1 || distanceToCell < shortestDistance
            yposition = successY(i) - floor(lidarRange * ydistanceToCell^2 / distanceToCell^2);
            xposition = successX(i) - floor(lidarRange * xdistanceToCell^2 / distanceToCell^2);
            shortestDistance = distanceToCell;
            successPosition = [yposition, xposition]; % Corrected to [y, x, z] format
        elseif distanceToCell == shortestDistance
            break;
        end
    end
end

function [successPosition, newDirection, newSegmentLength] = findNextPositionSpiral(currentPosition, lidarRange, lastDirection, lastSegmentLength)
    if isequal(lastDirection, [1, 0])
        newDirection = [0, 1];
        newSegmentLength = lastSegmentLength + lidarRange;
    elseif isequal(lastDirection, [0, 1])
        newDirection = [-1, 0];
        newSegmentLength = lastSegmentLength;
    elseif isequal(lastDirection, [-1, 0])
        newDirection = [0, -1];
        newSegmentLength = lastSegmentLength + lidarRange;
    else
        newDirection = [1, 0];
        newSegmentLength = lastSegmentLength;
    end

    successY = currentPosition(1) + newDirection(1) * newSegmentLength;
    successX = currentPosition(2) + newDirection(2) * newSegmentLength;
    successPosition = [successY, successX];
end

function proposedPath = generatePath(currentPosition, successPosition, uavElevation)
    xPath = linspace(currentPosition(2), successPosition(2), abs(currentPosition(2) - successPosition(2)) + 1);
    yPath = linspace(currentPosition(1), successPosition(1), abs(currentPosition(1) - successPosition(1)) + 1);
    xPathExtended = [xPath, repmat(successPosition(2), 1, length(yPath))];
    yPathExtended = [repmat(currentPosition(1), 1, length(xPath)), yPath];
    zPathExtended = repmat(uavElevation, 1, length(xPath) + length(yPath));
    proposedPath = [xPathExtended; yPathExtended; zPathExtended]'; % combine into a single path
    % proposedPath = unique(proposedPath,'stable','rows'); % remove any duplicate elements
end

function orientation = calculateOrientation(proposedPath, i)
    if i == 1 % first waypoint, orientation unchanged
        orientation = [1, 0, 0, 0]; % No rotation quaternion
    else
        deltaY = proposedPath(i, 1) - proposedPath(i-1, 1);
        deltaX = proposedPath(i, 2) - proposedPath(i-1, 2);
        yaw = atan2(deltaY, deltaX); % Calculate yaw angle
        % Convert yaw angle to quaternion
        w = cos(yaw / 2);
        x = 0; % No rotation around x-axis
        y = 0; % No rotation around y-axis
        z = sin(yaw / 2); % Rotation around z-axis
        orientation = [w, x, y, z];
    end
end

function targetFound = checkTargetFound(visited, targetRoi)
    targetFound = false;
    for y = targetRoi(1):targetRoi(2)
        for x = targetRoi(3):targetRoi(4)
            if visited(y, x) == 1
                targetFound = true;
            end
        end
    end
end

function updateHeatmaps(successGrid, distanceGrid, gridSize)
    figure(1); % update success heatmap
    imagesc(successGrid);
    colorbar;
    axis equal;
    title('Success Grid Heatmap');
    set(gca, 'YDir', 'normal');
    xlim([1 gridSize(1)]); xlabel('x (m)');
    ylim([1 gridSize(2)]); ylabel('y (m)');
    drawnow;

    figure(2); % update proximity heatmap
    imagesc(distanceGrid);
    colorbar;
    axis equal;
    title('Proximity Grid Heatmap');
    set(gca, 'YDir', 'normal');
    xlim([1 gridSize(1)]); xlabel('x (m)');
    ylim([1 gridSize(2)]); ylabel('y (m)');
    drawnow; % force display update
end

function visited = updateVisitedFromLidar(currentPosition, visited, gridSize, lidarRange)
    for y = 1:gridSize(2)
        for x = 1:gridSize(1)
            horizontalDistance = sqrt((currentPosition(2) - x)^2 + (currentPosition(1) - y)^2); % Corrected
            if horizontalDistance <= lidarRange
                visited(y, x) = 1;
            end
        end
    end
end

function lidarRange = maxLidarRange(elevationLimits, uavElevation)
    elevationRange = abs(deg2rad(elevationLimits(1) - elevationLimits(2)));
    lidarRange = tan(elevationRange) * abs(uavElevation);
end