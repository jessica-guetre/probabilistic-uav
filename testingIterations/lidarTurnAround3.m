% rng(0) % For repeatable results.
simTime = 30;    % in seconds
updateRate = 2;  % in Hz
color.Gray = 0.651*ones(1,3);
color.Red = [1 0 0];

% Create scenario.
scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200);
groundVertices = [-5 -5; -5 35; 35 35; 35 -5];
addMesh(scene,"polygon",{groundVertices,[-4 0]},color.Gray)

% Add the polygon mesh to the scene
targetX = 30*rand; % Random x-coordinate between 0 and 30
targetY = 30*rand; % Random y-coordinate between 0 and 30
targetSize = 2;
targetVertices = [targetX, targetY; targetX+targetSize, targetY; targetX+targetSize, targetY+targetSize; targetX, targetY+targetSize];
targetArea = [targetX, targetY, targetX+targetSize, targetY+targetSize];
addMesh(scene, "polygon", {targetVertices, [0 1]}, color.Red);

% Create platform.
velocity = [1 1 0];
plat = uavPlatform("UAV",scene,"InitialVelocity", velocity,"ReferenceFrame","ENU"); 
updateMesh(plat,"quadrotor",{1},[0 1 0],eul2tform([0 0 pi]));

% Define the lidar sensor
elevationLimits = [-20 20];
maxRange = 10;
lidarModel = uavLidarPointCloudGenerator("UpdateRate",updateRate,"ElevationLimits",elevationLimits,"MaxRange",maxRange,"HasOrganizedOutput",true);
lidar = uavSensor("Lidar",plat,lidarModel,MountingLocation=[0,0,-1]);

% Set up the 3D view of the scenario.
[ax, plotFrames] = show3D(scene);
plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.UAV.BodyFrame);
xlim([-5,35]);
ylim([-5,35]);
zlim([-4 10]);
hold on;

% Simulate lidar sensor.
colormap("jet")
pt = pointCloud(nan(1,1,3));
scatterplot = scatter3(nan,nan,nan,1,[0.3020 0.7451 0.9333],Parent=plotFrames.UAV.Lidar);
scatterplot.XDataSource = "reshape(pt.Location(:,:,1),[],1)";
scatterplot.YDataSource = "reshape(pt.Location(:,:,2),[],1)";
scatterplot.ZDataSource = "reshape(pt.Location(:,:,3),[],1)";
scatterplot.CDataSource = "reshape(pt.Location(:,:,3),[],1) - min(reshape(pt.Location(:,:,3),[],1))";

% Start simulation
setup(scene);
ptIdx = 0;
while advance(scene)
    ptIdx = ptIdx + 1;
    % Read lidar sensor data.
    [isUpdated,lidarSampleTime, pt] = read(lidar);
    if isUpdated
        % Extract X and Y coordinates of lidar points
        lidarX = pt.Location(:,:,1);
        lidarY = pt.Location(:,:,2);

        % Check if any lidar points fall within the polygon area
        targetDetected = any(lidarX >= targetArea(1) & lidarX <= targetArea(3) & ...
                              lidarY >= targetArea(2) & lidarY <= targetArea(4));
        if targetDetected
            disp('Polygon detected by Lidar!');
        end

        show3D(scene,"Time",lidarSampleTime,FastUpdate=true,Parent=ax);
        refreshdata
        drawnow limitrate
    end

    % Move UAV along its velocity vector.
    motion = read(plat);
    motion(1:2) = motion(1:2) + motion(4:5)/scene.UpdateRate;
    if any(motion(1:2) > 30) || any(motion(1:2) < -5)
        motion(4:5) = rand(1, 2) * 2 - 1; % Random new direction
    end
    move(plat, motion);

    % Update all sensors in the scene.
    updateSensors(scene)
end

hold off; 