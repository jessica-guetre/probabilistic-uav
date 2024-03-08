% script to create a customTerrain scene
clear scene;

% SASKATCHEWAN
% removeCustomTerrain("saskatchewan")
% clear skScene;
% [skScene, skR] = terrainScene("saskatchewan.dt2", "saskatchewan");
% scene = skScene;
% zBounds = [400 600];

% BRITISH COLUMBIA
removeCustomTerrain('britishcolumbia')
clear bcScene;
[bcScene, bcR] = terrainScene('britishcolumbia.dt2', 'britishcolumbia');
scene = bcScene;
% zBounds = [900 1100];

% ONTARIO
% removeCustomTerrain("ontario")
% clear onScene;
% [onScene, onR] = terrainScene("ontario.dt2", "ontario");
% scene = onScene;
% zBounds = [100 300];

figure(1); 
[ax,plotFrames] = show3D(scene);
hold on

figure(2); 
[ax,plotFrames] = show3D(scene);
xlim([1 bcR.RasterSize(1)])
ylim([1 bcR.RasterSize(2)])
% zlim(zBounds)
view([-30 20])
% axis equal
hold on