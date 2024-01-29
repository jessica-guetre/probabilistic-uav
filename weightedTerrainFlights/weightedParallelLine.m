filename = 'parallelLineGif.gif'; 

% Define terrain features
terrainFeatures = struct('Path', 1, 'River', 2, 'Tree', 3, 'Steep', 4, 'Empty', 5);

% Assign weights and colors to each terrain feature
weights = struct('Path', 5, 'River', 1, 'Tree', 2, 'Steep', 1, 'Empty', 3);
colors = struct('Path', [1 0 0], 'River', [0 0 1], 'Tree', [0 1 0], 'Steep', [0 0 0], 'Empty', [1 1 1], 'Target', [0.4940, 0.1840, 0.5560]);

% Create a grid (example: 10x10)
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

% Create lidar sensor.
lidarModel = uavLidarPointCloudGenerator("UpdateRate",updateRate,"MaxRange",maxRange,"AzimuthResolution",azimuthResolution,"ElevationLimits",elevationLimits,"ElevationResolution",elevationResolution,"HasOrganizedOutput",true);
lidar = uavSensor("Lidar",plat,lidarModel,MountingLocation=[0,0,-1]);

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
disp('gridScene.StopTime: ');
disp(gridScene.StopTime)

ptIdx = 0;
printCount = 0;
targetFound = false;
while gridScene.IsRunning
    ptIdx = ptIdx + 1;

    printCount = printCount + 1;
    if printCount >= 10
        disp('gridScene.CurrentTime: ');
        disp(gridScene.CurrentTime);
        printCount = 0;
    end

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
        figure(3)
        pcshow(tempPtc)

        if (~isempty(roiIndices) && targetFound == false)
            disp('Polygon detected by Lidar!');
            targetFound = true;
        end

        figure(1)
        show3D(gridScene,"Time",lidarSampleTime,"FastUpdate",true,"Parent",ax);
        xlim(xlimitsScene);
        ylim(ylimitsScene);
        zlim(zlimitsScene);
        
        refreshdata
        drawnow limitrate
    end

    % Show map building real time 
    figure(2)
    show(map3D);
    xlim(xlimitsScene);
    ylim(ylimitsScene);
    zlim(zlimitsScene);

    frame = getframe(1); 
    im = frame2im(frame); 
    [imind,cm] = rgb2ind(im,256); 
    if ptIdx == 1
        imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
    else
        imwrite(imind,cm,filename,'gif','WriteMode','append'); 
    end

    % Update the time and motion
    currentTime = gridScene.CurrentTime;
    motion = read(plat);

    % Check if UAV has reached the end of the current row
    if (motion(4) > 0 && motion(1) > (xlimitsScene(2) - 10)) || (motion(4) < 0 && motion(1) < (xlimitsScene(1) + 10))
        disp('Value change');
        turnAround = true;
        yStartingPosition = motion(2);
        xSpeed = motion(4);
        motion(4:5) = [0 maxSpeed];
    end

    if turnAround
        if motion(2) - yStartingPosition  >= rowHeight
            turnAround = false;
            motion(4:5) = [-xSpeed 0];
        end
    end

    disp('Current velocity motion(4:5) and position motion(1:2): ');
    disp(motion(4:5));
    disp(motion(1:2))
    disp('turnAround: ');
    disp(turnAround);

    % Move the UAV
    motion(1:2) = motion(1:2) + motion(4:5)/gridScene.UpdateRate;
    move(plat, motion);
    advance(gridScene);
    updateSensors(gridScene);
    
end