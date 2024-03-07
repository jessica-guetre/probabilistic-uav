initialPosition = [10 10 20];
gridSize = [100, 100];
terrainTypes = {'Type1', 'Type2', 'Type3'};
numIterations = 10;
allInputTerrain = [];
allInputPosition = [];
allWaypointsData = [];
allSimTimeData = [];

weights = rand(1, 3); % distance weight, probability grid weight, bias
learningRate = -0.05;
numEpochs = 10;
bestSim = [Inf weights]; % time, weights

for epoch = 1:numEpochs
    % fprintf('Epoch %d\n', epoch);

    allSimTimeData = [];
    
    for tType = terrainTypes
        selectedTerrainType = tType{1};
        
        for iter = 1:numIterations
            initialPosition = [10 10 20];
            [terrainInput, positionInput, waypointData, simTimeData] = runSimulation(selectedTerrainType, initialPosition, gridSize, weights);
            allSimTimeData = [allSimTimeData; simTimeData];
        end
        fprintf('Terrain %s, Average Sim Time %.2f\n', selectedTerrainType, mean(simTimeData));
    end

    avgSimTime = mean(allSimTimeData);
    weightUpdates = rand(1, 3);
    oldWeights = weights;

    if avgSimTime < bestSim(1)
        bestSim = [avgSimTime weights];
    else
        learningRate = learningRate * -1; % Change sign of learning rate if worse
        weights = bestSim(2:4); % Use best weights as starting point
    end

    for i = 1:length(weights)
        weights(i) = weights(i) + learningRate * weightUpdates(i);
    end

    fprintf('Epoch: %d, AvgSimTime: %.2f, BestSimTime: %.2f, New Weights: %.5f %.5f %.5f, Old Weights: %.5f %.5f %.5f, Weight Updates: %.5f %.5f %.5f\n', epoch, avgSimTime, bestSim(1), weights.', oldWeights.', weightUpdates.');
end

% save('UAVFlightSimulationData.mat', 'allInputTerrain', 'allInputPosition', 'allWaypointsData', 'allSimTimeData', 'successDistanceWeight', 'successTerrainWeight');

% --------------------------- HELPER FUNCTIONS ---------------------------

function [terrainInput, positionInput, waypointData, simTimeData] = runSimulation(selectedTerrainType, initialPosition, gridSize, weights)
    % Initialize parameters
    updateRate = 3; % in Hz
    simTime = 1000;
    maxRange = 30;
    azimuthResolution = 0.6;
    elevationResolution = 2.5;
    elevationLimits = [-90 -80];
    xlimitsScene = [0 100];
    ylimitsScene = [0 100];
    zlimitsScene = [-5 40];

    [terrainFeatures, probabilities, colors, probabilityGrid, featureVertices] = terrainSetup(gridSize, selectedTerrainType);
    gridScene = createUAVScenario(updateRate, simTime, gridSize);
    addTerrainMeshes(gridScene, featureVertices, colors);
    [targetPosition, roi] = createTarget(gridScene, probabilityGrid, colors, gridSize);
    [plat, lidar] = initializeUAVAndSensors(gridScene, initialPosition, updateRate, maxRange, azimuthResolution, elevationLimits, elevationResolution);
    [waypoints, orientation, waypointIndex] = determineWaypoints(gridSize, probabilityGrid, initialPosition, elevationLimits, initialPosition(3), weights);

    % COMMENT / UNCOMMENT TO SEE SIMULATION
    % figure(1)
    % [ax, plotFrames] = show3D(gridScene);
    % plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.UAV.BodyFrame);
    % xlim(xlimitsScene);
    % ylim(ylimitsScene);
    % zlim(zlimitsScene);
    % hold on;
    % colormap('jet');
    % ptc = pointCloud(nan(1,1,3));
    % scatterplot = scatter3(nan,nan,nan,1,[0.3020 0.7451 0.9333],"Parent",plotFrames.UAV.Lidar);
    % scatterplot.XDataSource = "reshape(ptc.Location(:,:,1), [], 1)";
    % scatterplot.YDataSource = "reshape(ptc.Location(:,:,2), [], 1)";
    % scatterplot.ZDataSource = "reshape(ptc.Location(:,:,3), [], 1)";
    % scatterplot.CDataSource = "reshape(ptc.Location(:,:,3), [], 1) - min(reshape(ptc.Location(:,:,3), [], 1))";
    % hold off;

    % SETUP SIMULATION LOOP
    % lidarSampleTime = [];
    pt = cell(1,((updateRate*simTime) +1)); 
    ptOut = cell(1,((updateRate*simTime) +1)); 
    map3D = occupancyMap3D(1);
    setup(gridScene); 
    ptIdx = 0;
    targetFound = false;
    while targetFound == false && ptIdx < waypointIndex
        ptIdx = ptIdx + 1;
        [isUpdated,lidarSampleTime,pt{ptIdx}] = read(lidar);
    
        if isUpdated
            sensorPose = getTransform(gridScene.TransformTree,"ENU","UAV/Lidar",lidarSampleTime);
            ptc = pt{ptIdx};
            ptOut{ptIdx} = removeInvalidPoints(pt{ptIdx});
            insertPointCloud(map3D,[sensorPose(1:3,4)' tform2quat(sensorPose)],ptOut{ptIdx},500);
            tform = affine3d(sensorPose');
            transformedPt = pctransform(pt{ptIdx}, tform);
            tempPtc = removeInvalidPoints(transformedPt);
            roiIndices = findPointsInROI(tempPtc,roi);
    
            if (~isempty(roiIndices) && targetFound == false)
                % disp('Polygon detected by Lidar!');
                targetFound = true;
                simTimeData = gridScene.CurrentTime;
            end
    
            % refreshdata(scatterplot, 'caller') % COMMENT THIS WHEN NOT PLOTTING LIDAR
            drawnow limitrate
        end
    
        % COMMENT / UNCOMMENT TO SEE SIMULATION
        % figure(2)
        % show(map3D);
        % xlim(xlimitsScene);
        % ylim(ylimitsScene);
        % zlim(zlimitsScene);
        % xlabel('x (m)','FontSize',14);
        % ylabel('y (m)','FontSize',14);
        % zlabel('z (m)','FontSize',14);

        move(plat,[waypoints(:,:,ptIdx+1),zeros(1,6),eul2quat(orientation(:,:,ptIdx+1)),zeros(1,3)]);
        advance(gridScene);
        updateSensors(gridScene);
    end

    terrainInput = reshape(probabilityGrid, [size(probabilityGrid,1), size(probabilityGrid,2), 1]);
    positionInput = initialPosition;
    waypointData = reshape(waypoints, [], 1);

    % fprintf('Simulation time: %16.f, number of waypoints %d\n', simTimeData, waypointIndex);
end

function [terrainFeatures, probabilities, colors, probabilityGrid, featureVertices] = terrainSetup(gridSize, selectedTerrainType)
    terrainFeatures = struct('Path', 1, 'River', 2, 'Tree', 3, 'Steep', 4, 'Empty', 5);
    probabilities = struct('Path', 1, 'River', 0.2, 'Tree', 0.4, 'Steep', 0.2, 'Empty', 0.6);
    colors = struct('Path', [1 0 0], 'River', [0 0 1], 'Tree', [0 1 0], 'Steep', [0 0 0], 'Empty', [1 1 1], 'Target', [0.4940, 0.1840, 0.5560]);
    probabilityGrid = probabilities.Empty * ones(gridSize);
    featureVertices = getFeatureVertices(selectedTerrainType, gridSize);
    for featureType = fieldnames(featureVertices)'
        featureName = featureType{1};
        featureArray = featureVertices.(featureName);
        
        for i = 1:length(featureArray)
            vertices = featureArray{i};
            mask = poly2mask(vertices(:,1), vertices(:,2), gridSize(1), gridSize(2));
            probabilityGrid(mask) = probabilities.(featureName);
        end
    end
end

function featureVertices = getFeatureVertices(terrainType, gridSize)
    featureVertices = struct();

    switch terrainType
        case 'Type1'
            featureVertices.Path = {[1, 30; gridSize(1), 30; gridSize(1), 35; 1, 35]};
            featureVertices.River = {
                [40, 1; 45, 1; 45, 30; 40, 30],
                [40, 35; 45, 35; 45, 90; 40, 90]
            };
            featureVertices.Tree = {
                [20, 1; 40, 1; 40, 10; 20, 10],
                [45, 1; gridSize(1), 1; gridSize(1), 10; 45, 10],
                [60, 10; gridSize(1), 10; gridSize(1), 20; 60, 20]
            };
            featureVertices.Steep = {
                [1, 98; gridSize(1), 98; gridSize(1), gridSize(2); 1, gridSize(2)],
                [10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]
            };
        case 'Type2'
            featureVertices.Path = {[1, 20; gridSize(1), 20; gridSize(1), 25; 1, 25]};
            featureVertices.River = {[1, 50; gridSize(1), 50; gridSize(1), 60; 1, 60]};
            featureVertices.Tree = {
                [20, 1; gridSize(1), 1; gridSize(1), 20; 20, 20],
                [20, 25; gridSize(1), 25; gridSize(1), 45; 20, 45]
            };
            featureVertices.Steep = {[10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]};
        case 'Type3'
            featureVertices.Path = {[10, 1; 20, 1; 20, gridSize(2); 10, gridSize(2)]};
            featureVertices.River = {[50, 1; 60, 1; 60, 30; 80, 30; 80, 70; 70, 70; 70, gridSize(2); 50, gridSize(2)]};
            featureVertices.Tree = {[60, 1; gridSize(1), 1; gridSize(1), 30; 60, 30]};
            featureVertices.Steep = {[70, 70; gridSize(1), 70; gridSize(1), gridSize(2); 70, gridSize(2)]};
    end
end

function gridScene = createUAVScenario(updateRate, simTime, gridSize)
    gridScene = uavScenario("UpdateRate", updateRate, "StopTime", simTime, "HistoryBufferSize", 200);
    gridScene.addInertialFrame("ENU", "MAP", trvec2tform([1 0 0]));
    addMesh(gridScene, "polygon", {[0 0; gridSize(1) 0; gridSize(1) gridSize(2); 0 gridSize(2)], [-5 0]}, 0.651*ones(1, 3));
end

function addTerrainMeshes(gridScene, featureVertices, colors)
    for featureType = fieldnames(featureVertices)'
        featureArray = featureVertices.(featureType{1});
        for i = 1:length(featureArray)
            vertices = featureArray{i};
            minX = min(vertices(:,1));
            maxX = max(vertices(:,1));
            minY = min(vertices(:,2));
            maxY = max(vertices(:,2));
            color = colors.(featureType{1});
            addMesh(gridScene, "polygon", {vertices, [0 1]}, color);
        end
    end
end

function [targetPosition, roi] = createTarget(gridScene, probabilityGrid, colors, gridSize)
    probabilities = probabilityGrid / sum(probabilityGrid, 'all');
    cumulativeProbabilities = cumsum(probabilities(:));
    targetCellIndex = find(cumulativeProbabilities >= rand(), 1);
    [targetRow, targetCol] = ind2sub([gridSize(1), gridSize(2)], targetCellIndex);
    targetPosition = [targetRow-1, targetCol-1];
    % targetPosition = [5, 22]; % TODO: REMOVE!
    targetVertices = [targetPosition; targetPosition + [1, 0]; targetPosition + [1, 1]; targetPosition + [0, 1]];
    targetZLimits = [0 3];
    addMesh(gridScene, "polygon", {targetVertices, targetZLimits}, colors.Target);
    roi = [targetPosition(1), targetPosition(1) + 2, targetPosition(2), targetPosition(2) + 2, targetZLimits(1), targetZLimits(2)];
end

function [plat, lidar] = initializeUAVAndSensors(gridScene, initialPosition, updateRate, maxRange, azimuthResolution, elevationLimits, elevationResolution)
    plat = uavPlatform("UAV",gridScene,"InitialPosition",initialPosition,"ReferenceFrame","ENU"); 
    updateMesh(plat,"quadrotor",{1.2},[1 0 0],eul2tform([0 0 pi]));
    lidarModel = uavLidarPointCloudGenerator("UpdateRate",updateRate,"MaxRange",maxRange,"AzimuthResolution",azimuthResolution,"ElevationLimits",elevationLimits,"ElevationResolution",elevationResolution,"HasOrganizedOutput",true);
    lidar = uavSensor("Lidar",plat,lidarModel,MountingLocation=[0,0,-1]);
end

function [waypoints, orientation, waypointIndex] = determineWaypoints(gridSize, probabilityGrid, initialPosition, elevationLimits, uavElevation, weights)
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


    while any(visited(:) == 0)
        successGrid = zeros(size(gridSize));
        for x = 1:gridSize(1)
            for y = 1:gridSize(2)
                if visited(x, y) == 0
                    distance = (abs(currentPosition(1) - x) + abs(currentPosition(2) - y)) / maxDistance;
                    terrainWeight = probabilityGrid(x, y);
                    successGrid(x, y) = (1/distance * weights(1)) + terrainWeight * weights(2) + weights(3);
                end
            end
        end
    
        maxSuccess = max(successGrid(:));
        [successX, successY] = find(successGrid == maxSuccess);
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
    
        % disp('successPosition: ');
        % disp(successPosition);

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