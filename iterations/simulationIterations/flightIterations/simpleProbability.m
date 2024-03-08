% filename = 'simpleProbability.gif';
updateRate = 3; % in Hz
simTime = 30;
gridSize = [100, 100];
initialPosition = [10 10 20];
maxRange = 30;
azimuthResolution = 0.6;
elevationResolution = 2.5;
elevationLimits = [-90 -80];
xlimitsScene = [0 100];
ylimitsScene = [0 100];
zlimitsScene = [-5 40];

% DEEP LEARNING DATA
flightData = struct('positions', [], 'terrainTypes', [], 'waypoints', []);

% TERRAIN SETUP
terrainFeatures = struct('Path', 1, 'River', 2, 'Tree', 3, 'Steep', 4, 'Empty', 5);
terrainFeatureNames = {'Path', 'River', 'Tree', 'Steep', 'Empty'};
weights = struct('Path', 5, 'River', 1, 'Tree', 2, 'Steep', 1, 'Empty', 3);
colors = struct('Path', [1 0 0], 'River', [0 0 1], 'Tree', [0 1 0], 'Steep', [0 0 0], 'Empty', [1 1 1], 'Target', [0.4940, 0.1840, 0.5560]);
weightedGrid = weights.Empty * ones(gridSize);
selectedTerrainType = 'Type3'; % 'Type1', 'Type2', 'Type3'
featureVertices = getFeatureVertices(selectedTerrainType, gridSize);

% CREATE UAV SCENARIO
gridScene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200);
gridScene.addInertialFrame("ENU","MAP",trvec2tform([1 0 0])); 
addMesh(gridScene,"polygon",{[0 0; gridSize(1) 0; gridSize(1) gridSize(2); 0 gridSize(2)],[-5 0]},0.651*ones(1,3));

% ADD TERRAIN MESHES TO UAV SCENARIO ACCORDING FEATURE VERTICES
for featureType = fieldnames(featureVertices)'
    featureArray = featureVertices.(featureType{1});

    for i = 1:length(featureArray)
        vertices = featureArray{i};

        minX = min(vertices(:,1));
        maxX = max(vertices(:,1));
        minY = min(vertices(:,2));
        maxY = max(vertices(:,2));

        weightedGrid(minX:maxX, minY:maxY) = weights.(featureType{1});
        
        color = colors.(featureType{1});
        addMesh(gridScene, "polygon", {vertices, [0 1]}, color);
    end
end

% CREATE TARGET BASED ON PROBABILITY AND ADD TO UAV SCENARIO
probabilities = weightedGrid / sum(weightedGrid, 'all');
cumulativeProbabilities = cumsum(probabilities(:));
targetCellIndex = find(cumulativeProbabilities >= rand(), 1);
[targetRow, targetCol] = ind2sub([gridSize(1), gridSize(2)], targetCellIndex);
targetPosition = [targetRow-1, targetCol-1];
targetVertices = [targetPosition; targetPosition + [1, 0]; targetPosition + [1, 1]; targetPosition + [0, 1]];
targetZLimits = [0 3];
addMesh(gridScene, "polygon", {targetVertices, targetZLimits}, colors.Target);
roi = [targetPosition(1), targetPosition(1) + 2, targetPosition(2), targetPosition(2) + 2, targetZLimits(1), targetZLimits(2)];

% CREATE UAV PLATFORM AND LIDAR SENSOR
plat = uavPlatform("UAV",gridScene,"InitialPosition",initialPosition,"ReferenceFrame","ENU"); 
updateMesh(plat,"quadrotor",{1.2},[1 0 0],eul2tform([0 0 pi]));
lidarModel = uavLidarPointCloudGenerator("UpdateRate",updateRate,"MaxRange",maxRange,"AzimuthResolution",azimuthResolution,"ElevationLimits",elevationLimits,"ElevationResolution",elevationResolution,"HasOrganizedOutput",true);
lidar = uavSensor("Lidar",plat,lidarModel,MountingLocation=[0,0,-1]);

% SETUP WAYPOINTs
maxNumWaypoints = gridSize(1) * gridSize(2) / 2;
waypoints = zeros(1, 3, maxNumWaypoints);
orientation = zeros(1, 3, maxNumWaypoints);
uavElevation = 20;
currentPosition = [10, 10, uavElevation];
visited = zeros(gridSize);
visited(currentPosition(1), currentPosition(2)) = 1;
waypointIndex = 1;
waypoints(1, :, waypointIndex) = currentPosition;
orientation(1, :, waypointIndex) = [0, 0, 0];
successDistanceWeight = 1;
successTerrainWeight = 1;

% DETERMINE WAYPOINTS WHILE THERE ARE STILL UNVISITED CELLS
while any(visited(:) == 0)
    successGrid = zeros(size(gridSize));
    for x = 1:gridSize(1)
        for y = 1:gridSize(2)
            if visited(x, y) == 0
                distanceSum = abs(currentPosition(1) - x) + abs(currentPosition(2) - y);
                terrainWeight = weightedGrid(x, y);
                successGrid(x, y) = (1/(distanceSum * successDistanceWeight)) * terrainWeight * successTerrainWeight;
            end
        end
    end

    maxSuccess = max(successGrid(:));
    [successX, successY] = find(successGrid == maxSuccess);
    minDistance = inf;
    successPosition = [successX(1), successY(1), uavElevation]; % Default to the first one

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
    % numNotVisited = sum(sum(visited == 0));
    % disp(['Number of cells not visited yet: ', num2str(numNotVisited)]);
end


% SETUP 3D VIEW
figure(1)
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

% SETUP SIMULATION LOOP
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
    [isUpdated,lidarSampleTime,pt{ptIdx}] = read(lidar);

    if isUpdated
        sensorPose = getTransform(gridScene.TransformTree,"ENU","UAV/Lidar",lidarSampleTime);
        ptc = pt{ptIdx};
        ptOut{ptIdx} = removeInvalidPoints(pt{ptIdx});
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

    % frame = getframe(2); 
    % im = frame2im(frame); 
    % [imind,cm] = rgb2ind(im,256); 
    % if ptIdx == 1
    %     imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    % else
    %     imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    % end

    if ptIdx + 1 <= waypointIndex
        move(plat,[waypoints(:,:,ptIdx+1),zeros(1,6),eul2quat(orientation(:,:,ptIdx+1)),zeros(1,3)]);
    else
        break;
    end

    % DEEP LEARNING DATA LOGGING
    currentTerrainType = weightedGrid(currentPosition(1), currentPosition(2)); % Assuming weightedGrid holds terrain types
    flightData.positions = [flightData.positions; currentPosition];
    flightData.terrainTypes = [flightData.terrainTypes; currentTerrainType];
    flightData.waypoints = [flightData.waypoints; waypoints(:,:,waypointIndex)];

    advance(gridScene);
    updateSensors(gridScene);
end

save('uavFlightData.mat', 'flightData');

