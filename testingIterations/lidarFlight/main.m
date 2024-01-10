
% Create scenario.
scene = createScene(xlimitsScene, ylimitsScene, zlimitsScene, updateRate, simTime);

% Create polygon target.
[scene, roi] = createTarget(scene, xboundsTarget, yboundsTarget, zlimitsTarget, sizeTarget);

% Create UAV platform.
[scene, plat] = createPlatform(scene, velocity, initialPosition);

% Create lidar sensor.
[plat, lidar] = createLidar(plat, updateRate, maxRange, azimuthResolution, elevationResolution, elevationLimits);

% Set up the 3D view of the scenario.
[ax, plotFrames] = show3D(scene);
plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.UAV.BodyFrame);
xlim(xlimitsScene);
ylim(ylimitsScene);
zlim([zlimitsScene(1) 8]);
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
targetFound = false;
while scene.IsRunning
    ptIdx = ptIdx + 1;

    % Read the simulated lidar data from the scenario
    [isUpdated,lidarSampleTime,pt{ptIdx}] = read(lidar);

    if isUpdated
        % Get Lidar sensor's pose relative to ENU reference frame.
        sensorPose = getTransform(scene.TransformTree, "ENU","UAV/Lidar",lidarSampleTime);

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
        zlim([zlimitsScene(1) 8]);
        
        refreshdata
        drawnow limitrate
    end

    % Show map building real time 
    figure(2)
    show(map3D);
    xlim(xlimitsScene);
    ylim(ylimitsScene);
    zlim([zlimitsScene(1) 8]);

    % Move UAV along its velocity vector.
    motion = read(plat);
    motion(1:2) = motion(1:2) + motion(4:5)/scene.UpdateRate;
    if any(motion(1:2) > 30) || any(motion(1:2) < -5)
        motion(4:5) = rand(1, 2) * 2 - 1; % Random new direction
    end
    move(plat, motion);

    advance(scene);
    updateSensors(scene); 
end