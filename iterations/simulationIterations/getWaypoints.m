function [waypoints, orientation, waypointIndex] = getWaypoints(gridSize, probabilityGrid, initialPosition, elevationLimits, uavElevation, weight)
    maxNumWaypoints = gridSize(1) * gridSize(2) / 2;
    waypoints = zeros(1, 3, maxNumWaypoints);
    orientation = zeros(1, 3, maxNumWaypoints);
    currentPosition = [initialPosition(1), initialPosition(2), uavElevation];
    visited = zeros(gridSize);
    visited(currentPosition(1), currentPosition(2)) = 1;
    waypointIndex = 1;
    waypoints(1, :, waypointIndex) = currentPosition;
    orientation(1, :, waypointIndex) = [0, 0, 0];

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
    end

    waypoints = waypoints(:,:,1:waypointIndex);
    orientation = orientation(:,:,1:waypointIndex);
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