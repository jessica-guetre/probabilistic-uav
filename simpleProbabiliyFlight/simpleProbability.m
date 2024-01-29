filename = 'simpleProbability.gif';

% Define terrain features
terrainFeatures = struct('Path', 1, 'River', 2, 'Tree', 3, 'Steep', 4, 'Empty', 5);
terrainFeatureNames = {'Path', 'River', 'Tree', 'Steep', 'Empty'};

% Assign weights and colors to each terrain feature
weights = struct('Path', 5, 'River', 1, 'Tree', 2, 'Steep', 1, 'Empty', 3);
colors = struct('Path', [1 0 0], 'River', [0 0 1], 'Tree', [0 1 0], 'Steep', [0 0 0], 'Empty', [1 1 1], 'Target', [0.4940, 0.1840, 0.5560]);

% Create a grid
gridSize = [100, 100];
terrainGrid = zeros(gridSize);

% Populate the grid with terrain features (example)
terrainGrid(:, :) = terrainFeatures.Empty;
terrainGrid(:, 98:100) = terrainFeatures.Steep;
terrainGrid(10:100, 90:98) = terrainFeatures.Steep;
terrainGrid(20:100, 1:10) = terrainFeatures.Tree;
terrainGrid(60:100, 10:20) = terrainFeatures.Tree;
terrainGrid(40:45, :) = terrainFeatures.River;
terrainGrid(:, 30:31) = terrainFeatures.Path;

% Create weighted and colored grids
weightedGrid = zeros(gridSize);
colorGrid = zeros(gridSize(1), gridSize(2), 3);  % For RGB colors

% Create the UAV scenario.
gridScene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200);
gridScene.addInertialFrame("ENU","MAP",trvec2tform([1 0 0])); 
addMesh(gridScene,"polygon",{[0 0; gridSize(1) 0; gridSize(1) gridSize(2); 0 gridSize(2)],[-5 0]},0.651*ones(1,3));

for r = 1:gridSize(1)
    for c = 1:gridSize(2)
        vertices = [r-1 c-1; r-1 c; r c; r c-1];
        switch terrainGrid(r, c)
            case terrainFeatures.Path
                weightedGrid(r, c) = weights.Path;
                addMesh(gridScene, "polygon", {vertices, [0 1]}, colors.Path);
            case terrainFeatures.River
                weightedGrid(r, c) = weights.River;
                addMesh(gridScene, "polygon", {vertices, [0 1]}, colors.River);
            case terrainFeatures.Tree
                weightedGrid(r, c) = weights.Tree;
                addMesh(gridScene, "polygon", {vertices, [0 1]}, colors.Tree);
            case terrainFeatures.Steep
                weightedGrid(r, c) = weights.Steep;
                addMesh(gridScene, "polygon", {vertices, [0 1]}, colors.Steep);
            otherwise
                weightedGrid(r, c) = weights.Empty;
        end
    end
end

% Create target and add to the gridScene
probabilities = weightedGrid / sum(weightedGrid, 'all');
cumulativeProbabilities = cumsum(probabilities(:));
targetCellIndex = find(cumulativeProbabilities >= rand(), 1);
[targetRow, targetCol] = ind2sub([gridSize(1), gridSize(2)], targetCellIndex);
targetPosition = [targetRow-1, targetCol-1];
targetVertices = [targetPosition; targetPosition + [1, 0]; targetPosition + [1, 1]; targetPosition + [0, 1]];
if terrainGrid(targetVertices) == terrainFeatures.Empty
    targetZLimits = [0 2];
else
    targetZLimits = [1 3];
end
addMesh(gridScene, "polygon", {targetVertices, targetZLimits}, colors.Target);

% Define target ROI.
roi = [targetPosition(1), targetPosition(1) + 2, targetPosition(2), targetPosition(2) + 2, 0, 2];

% Create UAV platform.
plat = uavPlatform("UAV",gridScene,"InitialVelocity",initialVelocity,"InitialPosition",initialPosition,"ReferenceFrame","ENU"); 
updateMesh(plat,"quadrotor",{1.2},[1 0 0],eul2tform([0 0 pi]));

% Parallel line search pattern
turnAround = false; % Flag to indicate turn around

% Create lidar sensor
lidarModel = uavLidarPointCloudGenerator("UpdateRate",updateRate,"MaxRange",maxRange,"AzimuthResolution",azimuthResolution,"ElevationLimits",elevationLimits,"ElevationResolution",elevationResolution,"HasOrganizedOutput",true);
lidar = uavSensor("Lidar",plat,lidarModel,MountingLocation=[0,0,-1]);

% Pre-allocate the waypoints and orientation array
maxNumWaypoints = gridSize(1) * gridSize(2) / 2;
waypoints = zeros(1, 3, maxNumWaypoints);
orientation = zeros(1, 3, maxNumWaypoints);

% Initialize variables for the loop
uavElevation = 20;
currentPosition = [10, 10, uavElevation];
visited = zeros(size(terrainGrid));
visited(currentPosition(1), currentPosition(2)) = 1;
waypointIndex = 1;
waypoints(1, :, waypointIndex) = currentPosition;
orientation(1, :, waypointIndex) = [0, 0, 0];

% Weights to be adjusted
successDistanceWeight = 1;
successTerrainWeight = 1;

% While there are unvisited cells
while any(visited(:) == 0)
    % Calculate the success of travelling to each cell
    successGrid = zeros(size(terrainGrid));
    for x = 1:gridSize(1)
        for y = 1:gridSize(2)
            if visited(x, y) == 0
                distanceSum = abs(currentPosition(1) - x) + abs(currentPosition(2) - y);
                terrainType = terrainFeatureNames{terrainGrid(x, y)};
                successGrid(x, y) = (1/(distanceSum * successDistanceWeight)) * weights.(terrainType) * successTerrainWeight;
            end
        end
    end

    % Find the cells with the highest success values
    maxSuccess = max(successGrid(:));
    [successX, successY] = find(successGrid == maxSuccess);

    % Set min distance and success position
    minDistance = inf;
    successPosition = [successX(1), successY(1), uavElevation]; % Default to the first one

    % Check which cell is closest if there are several choices
    for i = 1:length(successX)
        successDistance = abs(successX(i) - currentPosition(1)) + abs(successY(i) - currentPosition(2));
        if successDistance < minDistance
            minDistance = successDistance;
            successPosition = [successX(i), successY(i), 20];
        elseif successDistance == minDistance
            break;
        end
    end

    disp('successPosition: ');
    disp(successPosition);

    % Calculate the range of x and y positions the UAV must occupy to get from current to success
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

    % Add each point in the proposed path to waypoints and calculate orientation
    for i = 1:size(proposedPath, 1)
        waypointIndex = waypointIndex + 1;
        waypoints(1, :, waypointIndex) = proposedPath(i, :);

        % Calculate orientation based on movement direction
        if i > 1
            dx = proposedPath(i, 1) - proposedPath(i-1, 1);
            dy = proposedPath(i, 2) - proposedPath(i-1, 2);
            yaw = atan2(dy, dx);
            orientation(1, :, waypointIndex) = [yaw, 0, 0];
        else
            % Keep the same orientation for the first point
            orientation(1, :, waypointIndex) = orientation(1, :, waypointIndex - 1);
        end
    end

    % Update the current position
    currentPosition = successPosition;

    % Update visited cells based on Lidar range along the proposedPath
    visited = updateVisitedFromLidar(proposedPath, visited, gridSize, elevationLimits, uavElevation);

    % % Display the number of cells not visited yet
    % numNotVisited = sum(sum(visited == 0));
    % disp(['Number of cells not visited yet: ', num2str(numNotVisited)]);
end


% Set up the 3D view of the scenario.
[ax, plotFrames] = show3D(gridScene);
plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.UAV.BodyFrame);
xlim(xlimitsScene);
ylim(ylimitsScene);
zlim(zlimitsScene);
hold on;

colormap('jet');
ptc = pointCloud(nan(1,1,3));
scatterplot = scatter3(nan,nan,nan,1,[0.3020 0.7451 0.9333],...
    "Parent",plotFrames.UAV.Lidar);
scatterplot.XDataSource = "reshape(ptc.Location(:,:,1), [], 1)";
scatterplot.YDataSource = "reshape(ptc.Location(:,:,2), [], 1)";
scatterplot.ZDataSource = "reshape(ptc.Location(:,:,3), [], 1)";
scatterplot.CDataSource = "reshape(ptc.Location(:,:,3), [], 1) - min(reshape(ptc.Location(:,:,3), [], 1))";
hold off;

lidarSampleTime = [];
pt = cell(1,((updateRate*simTime) +1)); 
ptOut = cell(1,((updateRate*simTime) +1)); 

map3D = occupancyMap3D(1);
setup(gridScene); 

ptIdx = 0;
printCount = 0;
targetFound = false;
while gridScene.IsRunning
    ptIdx = ptIdx + 1;

    % Read the simulated lidar data from the scenario
    [isUpdated,lidarSampleTime,pt{ptIdx}] = read(lidar);

    if isUpdated
        % Get Lidar sensor's pose relative to ENU reference frame.
        sensorPose = getTransform(gridScene.TransformTree,"ENU","UAV/Lidar",lidarSampleTime);

        % Process the simulated Lidar pointcloud.
        ptc = pt{ptIdx};
        ptOut{ptIdx} = removeInvalidPoints(pt{ptIdx});

        % Construct the occupancy map using Lidar readings.
        insertPointCloud(map3D,[sensorPose(1:3,4)' tform2quat(sensorPose)],ptOut{ptIdx},500);

        % Transform point cloud 
        tform = affine3d(sensorPose');
        transformedPt = pctransform(pt{ptIdx}, tform);
        tempPtc = removeInvalidPoints(transformedPt);
        roiIndices = findPointsInROI(tempPtc,roi);
        % figure(3)
        % pcshow(tempPtc)

        if (~isempty(roiIndices) && targetFound == false)
            disp('Polygon detected by Lidar!');
            targetFound = true;
            % break;
        end

        % figure(1)
        % show3D(gridScene,"Time",lidarSampleTime,"FastUpdate",true,"Parent",ax);
        % xlim(xlimitsScene);
        % ylim(ylimitsScene);
        % zlim(zlimitsScene);

        refreshdata
        drawnow limitrate
    end

    % Show map building real time 
    figure(2)
    show(map3D);
    xlim(xlimitsScene);
    ylim(ylimitsScene);
    zlim(zlimitsScene);

    frame = getframe(2); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    if ptIdx == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end

    % Move the UAV efficiently
    if ptIdx + 1 <= waypointIndex
        move(plat,[waypoints(:,:,ptIdx+1),zeros(1,6),eul2quat(orientation(:,:,ptIdx+1)),zeros(1,3)]);
    else
        break;
    end

    % Efficient sensor updates
    advance(gridScene);
    updateSensors(gridScene);
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
