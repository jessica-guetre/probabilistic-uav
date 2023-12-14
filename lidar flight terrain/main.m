% script to create a customTerrain scene
clear scene;

% BRITISH COLUMBIA
removeCustomTerrain('britishcolumbia')
clear bcScene;
[bcScene, bcR] = terrainScene('britishcolumbia.dt2', 'britishcolumbia');
scene = bcScene;
zBounds = [900 1100];

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

figure(2); 
[ax,plotFrames] = show3D(scene);
xlim([0 500])
ylim([0 500])
zlim(zBounds)
view([-30 20])
% axis equal
hold on

