% Map Environment For Motion Planning Using UAV Lidar
% UAV Toolbox User Guide p. 176
% https://www.mathworks.com/help/uav/ug/map-environment-motion-planning-using-uav-lidar.html

% UAV equipped with lidar sensor used for 3D mapping in an environment.
% Generated occupancy map aids in motion planning for the UAV.
% UAV follows a predefined trajectory, utilizing known poses for mapping.
% Point clouds from lidar and UAV position contribute to building the environment map.
% The mapped environment facilitates motion planning for delivering packages in an apartment complex.


% CREATE SCENARIO
close all
close all hidden
simTime = 60;    % in seconds
updateRate = 2;  % in Hz
scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime);

% Floor
addMesh(scene, "Polygon",{[0 0;80 0;80 80;40 80;40 40;0 40],[-1 0]},[0.3 0.3 0.3]);
% Features
addMesh(scene, "Polygon",{[10 0;30 0;30 20;10 20],[0 40]},[0.4660 0.6740 0.1880]);
addMesh(scene, "Polygon",{[45 0;80 0;80 30;45 30],[0 60]},[0.9290 0.6980 0.1250]);
addMesh(scene, "Polygon",{[0 35;10 35;10 40;0 40],[0 5]},[0 0.5 0]);
addMesh(scene, "Polygon",{[50 40;80 40;80 70;50 70],[0 5]},[0 0.4470 0.7410]);
addMesh(scene, "Polygon",{[0 0;2 0;2 4;0 4],[0 3]},[0.6350 0.0780 0.1840]);

show3D(scene);
axis equal
view([-115 20])

% CREATE MAPPING TRAJECTORY
% Waypoints
x = -20:80;
y = -20:80;
z = 100*ones(1,length(x));
waypoints = [x' y' z'];

orientation_eul = [0 0 0];
orientation_quat = quaternion(eul2quat(orientation_eul));
orientation_vec = repmat(orientation_quat,length(x),1);

time = 0:(simTime/(length(x)-1)):simTime;

trajectory = waypointTrajectory("Waypoints",waypoints,"Orientation",orientation_vec, ...
    "SampleRate",updateRate,"ReferenceFrame","ENU","TimeOfArrival",time); 

initial_pose = [-20 -20 100 1 0 0 0]; 

plat = uavPlatform("UAV",scene,"Trajectory",trajectory,"ReferenceFrame","ENU"); 
updateMesh(plat,"quadrotor",{4},[1 0 0],eye(4));

lidarmodel = uavLidarPointCloudGenerator("AzimuthResolution",0.6, ...
    "ElevationLimits",[-90 -20],"ElevationResolution",2.5, ...
    "MaxRange",200,"UpdateRate",2,"HasOrganizedOutput",true);

lidar = uavSensor("Lidar",plat,lidarmodel,"MountingLocation",[0 0 -1],"MountingAngles",[0 0 0]); 

[ax,plotFrames] = show3D(scene);
xlim([-15 80]);
ylim([-15 80]);
zlim([0 80]);
view([-115 20]); 
axis equal 
hold on