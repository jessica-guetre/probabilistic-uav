% removeCustomTerrain('britishcolumbia')
clear scene;
clear plat;
clear lidar;
% removeCustomTerrain('britishcolumbia')

% % Scene constants
% dtedFile = 'britishcolumbia.dt2';
% terrainName = 'britishcolumbia';
% terrainColor = [0.4660 0.6740 0.1880];
% % Read in DTED file.
% [dataGrid, spatialReference] = readgeoraster(dtedFile);
% xlimits = [1 spatialReference.RasterSize(1)];
% ylimits = [1 spatialReference.RasterSize(2)];
% % Add custom terrain from DTED file.
% addCustomTerrain(terrainName, dtedFile);
% % Create a UAV scene with a reference location specified in the center of the terrain.
% refLocation = [mean(spatialReference.LatitudeLimits), mean(spatialReference.LongitudeLimits), 0];
% scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200, "ReferenceLocation", refLocation);
% % scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200, "ReferenceLocation", "ENU");
vertices = [xlimitsScene(1) ylimitsScene(1); xlimitsScene(2) ylimitsScene(1); xlimitsScene(2) ylimitsScene(2); xlimitsScene(1) ylimitsScene(2)];
scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200);
addMesh(scene,"polygon",{vertices,[920 940]},0.651*ones(1,3));

% Create polygon target.
[scene, roi] = createTarget(scene, xboundsTarget, yboundsTarget, zlimitsTarget, sizeTarget, [100 100]);

% Create UAV platform.
[scene, plat] = createPlatform(scene, initialVelocity, initialPosition);

% Spiral Search Pattern
directions = [1 0; 0 1; -1 0; 0 -1]; % Right, Up, Left, Down
currentDirectionIndex = 1;
nextDirectionChangeTime = segmentLength / maxSpeed;
disp('nextDirectionChangeTime: ');
disp(nextDirectionChangeTime);

% % Add the terrain mesh to the scene with the specified color.
% addMesh(scene, "terrain", {terrainName, xlimits, ylimits}, terrainColor);

% Create lidar sensor.
[plat, lidar] = createLidar(plat, updateRate, maxRange, azimuthResolution, elevationResolution, elevationLimits);

% Set up the 3D view of the scenario.
[ax, plotFrames] = show3D(scene);
% plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.UAV.BodyFrame);
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

setup(scene); 
disp('scene.StopTime: ');
disp(scene.StopTime)

ptIdx = 0;
printCount = 0;
targetFound = false;
while scene.IsRunning
    ptIdx = ptIdx + 1;

    printCount = printCount + 1;
    if printCount >= 10
        disp('scene.CurrentTime: ');
        disp(scene.CurrentTime);
        printCount = 0;
    end

    % Read the simulated lidar data from the scenario
    [isUpdated,lidarSampleTime,pt{ptIdx}] = read(lidar);

    if isUpdated
        % Get Lidar sensor's pose relative to ENU reference frame.
        sensorPose = getTransform(scene.TransformTree,"ENU","UAV/Lidar",lidarSampleTime);

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
        show3D(scene,"Time",lidarSampleTime,"FastUpdate",true,"Parent",ax);
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

    % Update the time and motion
    currentTime = scene.CurrentTime;
    motion = read(plat);

    % Check if it's time to change direction
    if currentTime >= nextDirectionChangeTime
        % Increase segment length at the end of each loop
        disp('currentDirectionIndex: ');
        disp(currentDirectionIndex);
        if currentDirectionIndex == 4 ||  currentDirectionIndex == 2
            segmentLength = segmentLength + segmentStep;
        end
        
        % Update direction
        currentDirectionIndex = mod(currentDirectionIndex, 4) + 1;
        motion(4:5) = maxSpeed * directions(currentDirectionIndex, :);
        
        % Schedule next direction change
        nextDirectionChangeTime = currentTime + segmentLength / maxSpeed;
        disp('Updated velocity motion(4:5): ');
        disp(motion(4:5));
        disp('nextDirectionChangeTime: ');
        disp(nextDirectionChangeTime);
        disp('segmentLength: ');
        disp(segmentLength);
    end

    disp('Current velocity motion(4:5) and position motion(1:2): ');
    disp(motion(4:5));
    disp(motion(1:2))

    % Move the UAV
    motion(1:2) = motion(1:2) + motion(4:5)/scene.UpdateRate;
    move(plat, motion);
    advance(scene);
    updateSensors(scene); 
    
end