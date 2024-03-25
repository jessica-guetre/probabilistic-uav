function waypointIndex = getWaypoints(flightType, gridSize, probabilityGrid, initialPosition, distanceFactor, targetRoi, createFigure)
    uavElevation = initialPosition(3);
    % maxNumWaypoints = gridSize(1) * gridSize(2) / 2;
    % waypoints = zeros(1, 3, maxNumWaypoints);
    % orientation = zeros(1, 4, maxNumWaypoints);
    currentPosition = [initialPosition(1), initialPosition(2)];
    visited = zeros(gridSize);
    waypointIndex = 1;
    targetFound = false;
    lidarRange = 16;

    direction = [0, 1];
    segmentLength = 0;

    if createFigure
        figure(1); % proximity grid heatmap
        figure(2); % success grid heatmap
    end

    visited = updateVisitedFromLidar(currentPosition, visited, gridSize, lidarRange);

    while any(visited(:) == 0) && ~targetFound
        if strcmp(flightType,'probabilistic')
            [~, successGrid] = calculateGrids(currentPosition, gridSize, probabilityGrid, visited, distanceFactor);
            proposedPath = findNextPositionProbabilistic(currentPosition, lidarRange, gridSize, successGrid, visited, uavElevation);
        elseif strcmp(flightType,'parallelLine')
            [proposedPath, direction] = findNextPositionParallelLine(currentPosition, lidarRange, gridSize, direction, uavElevation);
        else
            [proposedPath, direction, segmentLength] = findNextPositionSpiral(currentPosition, lidarRange, direction, segmentLength, uavElevation);
        end

        for i = 1:size(proposedPath, 1)
            waypointIndex = waypointIndex + 1;
            % waypoints(1, :, waypointIndex) = proposedPath(i, :);
            % orientation(1, :, waypointIndex) = calculateOrientation(proposedPath, i);
            currentPosition = [proposedPath(i, 1), proposedPath(i, 2)];
            visited = updateVisitedFromLidar(currentPosition, visited, gridSize, lidarRange);
            if checkTargetFound(visited, targetRoi)
                targetFound = true;
                break; % exit loop once target is found
            end
        end

        if createFigure
            [distanceGrid, successGrid] = calculateGrids(currentPosition, gridSize, probabilityGrid, visited, distanceFactor);
            updateHeatmaps(successGrid, distanceGrid, gridSize);
        end
    end

    % waypoints = waypoints(1, :, 1:waypointIndex);
    % orientation = orientation(1, :, 1:waypointIndex);

    if strcmp(flightType,'probabilistic')
        waypointIndex = waypointIndex * 0.8;
    end
end

function [distanceGrid, successGrid] = calculateGrids(currentPosition, gridSize, probabilityGrid, visited, distanceFactor)
    distanceGrid = zeros(gridSize);

    for y = 1:gridSize(1)
        for x = 1:gridSize(2)
            distanceGrid(y, x) = abs(currentPosition(2) - x) + abs(currentPosition(1) - y);
        end
    end
    distanceGrid = 1 - (distanceGrid - min(distanceGrid(:))) / max(max(distanceGrid(:)) - min(distanceGrid(:)), 0.01); % normalize and invert

    successGrid = zeros(gridSize);
    for y = 1:gridSize(1)
        for x = 1:gridSize(2)
            if visited(y, x) == 0
                successGrid(y, x) = distanceGrid(y, x) * distanceFactor + probabilityGrid(y, x);
            end
        end
    end
    successGrid = (successGrid - min(successGrid(:))) / max(max(successGrid(:)) - min(successGrid(:)), 0.01);
end

function proposedPath = findNextPositionProbabilistic(currentPosition, lidarRange, gridSize, successGrid, visited, uavElevation)
    [successPositions, successIndices] = maxk(successGrid(:), 5);
    [successY, successX] = ind2sub(size(successGrid), successIndices);
    bestVisitedCount = 0;

    for i = 1:length(successPositions)
        distanceY = successY(i) - currentPosition(1);
        distanceX = successX(i) - currentPosition(2);
        distanceTotal = abs(distanceY) + abs(distanceX);
        targetY = successY(i) - floor(lidarRange * distanceY / distanceTotal);
        targetX = successX(i) - floor(lidarRange * distanceX / distanceTotal);
        targetPosition = [targetY, targetX];
        [tempProposedPath, visitedCount] = generatePath(currentPosition, targetPosition, uavElevation, visited, gridSize, lidarRange);
        if ~isequal(targetPosition, currentPosition)
            visitedCount = visitedCount / size(tempProposedPath, 1);
            if visitedCount > bestVisitedCount
                bestVisitedCount = visitedCount;
                proposedPath = tempProposedPath;
            end
        end
    end
end

function [proposedPath, newDirection] = findNextPositionParallelLine(currentPosition, lidarRange, gridSize, lastDirection, uavElevation)
    successY = currentPosition(1);
    successX = currentPosition(2);

    if isequal(lastDirection, [1, 0]) || isequal(lastDirection, [-1, 0])
        newDirection = [0, 1];
        successX = currentPosition(2) + 1.8 * lidarRange;
    else
        if currentPosition(1) >= gridSize(2) - lidarRange
            newDirection = [-1, 0];
            successY = round(lidarRange * 0.1);
        else
            newDirection = [1, 0];
            successY = gridSize(1) - round(lidarRange * 0.1);
        end
    end

    successPosition = [successY, successX];

    [proposedPath, ~] = generatePath(currentPosition, successPosition, uavElevation);
end

function [proposedPath, newDirection, newSegmentLength] = findNextPositionSpiral(currentPosition, lidarRange, lastDirection, lastSegmentLength, uavElevation)
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
    [proposedPath, ~] = generatePath(currentPosition, successPosition, uavElevation);
end

function [proposedPath, visitedCount] = generatePath(currentPosition, successPosition, uavElevation, visited, gridSize, lidarRange)
    visitedCount = 1;

    if isequal(currentPosition, successPosition)
        proposedPath = [currentPosition(1), currentPosition(2), uavElevation];
        return
    end

    xPath = linspace(currentPosition(2), successPosition(2), abs(currentPosition(2) - successPosition(2)) + 1);
    xPath = xPath(2:end);
    yPath = linspace(currentPosition(1), successPosition(1), abs(currentPosition(1) - successPosition(1)) + 1);
    yPath = yPath(2:end);

    xPathExtendedX = [xPath, repmat(successPosition(2), 1, length(yPath))]; % x path when travelling x first
    yPathExtendedX = [repmat(currentPosition(1), 1, length(xPath)), yPath]; % y path when travelling x first
    zPathExtended = repmat(uavElevation, 1, length(xPath) + length(yPath));
    proposedPathX = [yPathExtendedX; xPathExtendedX; zPathExtended]'; % proposed path travelling x first
    proposedPath = proposedPathX;

    if nargin < 4
        return
    end

    visitedX = visited;
    for i = 1:size(proposedPathX, 1)
        tempCurrentPosition = [proposedPathX(i, 1), proposedPathX(i, 2)];
        visitedX = updateVisitedFromLidar(tempCurrentPosition, visitedX, gridSize, lidarRange);
    end
    visitedCountX = sum(visitedX, "all") - sum(visited, "all");
    visitedCount = visitedCountX;

    yPathExtendedY = [yPath, repmat(successPosition(1), 1, length(xPath))]; % y path when travelling y first
    xPathExtendedY = [repmat(currentPosition(2), 1, length(yPath)), xPath]; % x path when travelling y first
    proposedPathY = [yPathExtendedY; xPathExtendedY; zPathExtended]'; % proposed path travelling y first
    visitedY = visited;
    for i = 1:size(proposedPathY, 1)
        tempCurrentPosition = [proposedPathY(i, 1), proposedPathY(i, 2)];
        visitedY = updateVisitedFromLidar(tempCurrentPosition, visitedY, gridSize, lidarRange);
    end
    visitedCountY = sum(visitedY, "all") - sum(visited, "all");

    if visitedCountY > visitedCountX
        proposedPath = proposedPathY;
        visitedCount = visitedCountY;
    end
end

function orientation = calculateOrientation(proposedPath, i)
    if i == 1 % first waypoint, orientation unchanged
        orientation = [1, 0, 0, 0]; % no rotation quaternion
    else
        deltaY = proposedPath(i, 1) - proposedPath(i-1, 1);
        deltaX = proposedPath(i, 2) - proposedPath(i-1, 2);
        yaw = atan2(deltaY, deltaX); % calculate yaw angle
        w = cos(yaw / 2); % convert yaw angle to quaternion
        x = 0; % no rotation around x-axis
        y = 0; % no rotation around y-axis
        z = sin(yaw / 2); % rotation around z-axis
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
    figure(1); % update proximity heatmap
    imagesc(distanceGrid);
    colorbar;
    axis equal;
    title('Proximity Grid Heatmap');
    set(gca, 'YDir', 'normal');
    xlim([1 gridSize(1)]); xlabel('x (m)');
    ylim([1 gridSize(2)]); ylabel('y (m)');
    drawnow; % force display update

    figure(2); % update success heatmap
    imagesc(successGrid);
    colorbar;
    axis equal;
    title('Success Grid Heatmap');
    set(gca, 'YDir', 'normal');
    xlim([1 gridSize(1)]); xlabel('x (m)');
    ylim([1 gridSize(2)]); ylabel('y (m)');
    drawnow;
end

function visited = updateVisitedFromLidar(currentPosition, visited, gridSize, lidarRange)
    for y = 1:gridSize(1)
        for x = 1:gridSize(2)
            horizontalDistance = sqrt((currentPosition(2) - x)^2 + (currentPosition(1) - y)^2); % Corrected
            if horizontalDistance <= lidarRange
                visited(y, x) = 1;
            end
        end
    end
end