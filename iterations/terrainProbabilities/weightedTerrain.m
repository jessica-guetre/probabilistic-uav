initialPosition = [0 0 20];
gridSize = [100, 200];
selectedTerrainType = 'Type3'; % {'Type1', 'Type2', 'Type3'}
updateRate = 3; % in Hz
simTime = 1000;
maxRange = 30;
azimuthResolution = 0.6;
elevationResolution = 2.5;
elevationLimits = [-90 -80];
xlimitsScene = [0 gridSize(1)];
ylimitsScene = [0 gridSize(2)];
zlimitsScene = [-5 40];

gridScene = createUAVScenario(updateRate, simTime, gridSize);
[terrainFeatures, weights, colors, weightedGrid, featureVertices] = terrainSetup(gridSize, selectedTerrainType);
addTerrainMeshes(gridScene, featureVertices, colors);
[targetPosition, roi] = createTarget(gridScene, weightedGrid, colors, gridSize);
[plat, lidar] = initializeUAVAndSensors(gridScene, initialPosition, updateRate, maxRange, azimuthResolution, elevationLimits, elevationResolution);

[ax, plotFrames] = show3D(gridScene);
ax.FontSize = 13; 
plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.UAV.BodyFrame);
xlabel('x (m)');
ylabel('y (m)');
zlabel('z (m)');
xlim(xlimitsScene);
ylim(ylimitsScene);
zlim(zlimitsScene);

function [terrainFeatures, weights, colors, weightedGrid, featureVertices] = terrainSetup(gridSize, selectedTerrainType)
    terrainFeatures = struct('Trail', 1, 'Water', 2, 'Forest', 3, 'Elevation', 4, 'Field', 5);
    weights = struct('Trail', 5, 'Water', 1, 'Forest', 2, 'Elevation', 1, 'Field', 3);
    colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667], 'Target', [1 0 0]);
    weightedGrid = weights.Field * ones(gridSize);
    featureVertices = getFeatureVertices(selectedTerrainType, gridSize);
    for featureType = fieldnames(featureVertices)'
        featureName = featureType{1};
        featureArray = featureVertices.(featureName);
        
        for i = 1:length(featureArray)
            vertices = featureArray{i};
            mask = poly2mask(vertices(:,1), vertices(:,2), gridSize(1), gridSize(2));
            weightedGrid(mask) = weights.(featureName);
        end
    end
end

% function featureVertices = getFeatureVertices(terrainType, gridSize)
%     featureVertices = struct();
% 
%     switch terrainType
%         case 'Type1'
%             featureVertices.Trail = {[1, 30; gridSize(1), 30; gridSize(1), 35; 1, 35]};
%             featureVertices.Water = {
%                 [40, 1; 45, 1; 45, 30; 40, 30],
%                 [40, 35; 45, 35; 45, 90; 40, 90]
%             };
%             featureVertices.Forest = {
%                 [20, 1; 40, 1; 40, 10; 20, 10],
%                 [45, 1; gridSize(1), 1; gridSize(1), 10; 45, 10],
%                 [60, 10; gridSize(1), 10; gridSize(1), 20; 60, 20]
%             };
%             featureVertices.Elevation = {
%                 [1, 98; gridSize(1), 98; gridSize(1), gridSize(2); 1, gridSize(2)],
%                 [10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]
%             };
%         case 'Type2'
%             featureVertices.Trail = {[1, 20; gridSize(1), 20; gridSize(1), 25; 1, 25]};
%             featureVertices.Water = {[1, 50; gridSize(1), 50; gridSize(1), 60; 1, 60]};
%             featureVertices.Forest = {
%                 [20, 1; gridSize(1), 1; gridSize(1), 20; 20, 20],
%                 [20, 25; gridSize(1), 25; gridSize(1), 45; 20, 45]
%             };
%             featureVertices.Elevation = {[10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]};
%         case 'Type3'
%             featureVertices.Trail = {[10, 1; 20, 1; 20, gridSize(2); 10, gridSize(2)]};
%             featureVertices.Water = {[50, 1; 60, 1; 60, 30; 80, 30; 80, 70; 70, 70; 70, gridSize(2); 50, gridSize(2)]};
%             featureVertices.Forest = {[60, 1; gridSize(1), 1; gridSize(1), 30; 60, 30]};
%             featureVertices.Elevation = {[70, 70; gridSize(1), 70; gridSize(1), gridSize(2); 70, gridSize(2)]};
%     end
% end

function gridScene = createUAVScenario(updateRate, simTime, gridSize)
    gridScene = uavScenario("UpdateRate", updateRate, "StopTime", simTime, "HistoryBufferSize", 200);
    gridScene.addInertialFrame("ENU", "MAP", trvec2tform([1 0 0]));
    addMesh(gridScene, "polygon", {[0 0; gridSize(1) 0; gridSize(1) gridSize(2); 0 gridSize(2)], [-5 0]}, [0.4667 0.6745 0.1882]);
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

function [targetPosition, roi] = createTarget(gridScene, weightedGrid, colors, gridSize)
    probabilities = weightedGrid / sum(weightedGrid, 'all');
    cumulativeProbabilities = cumsum(probabilities(:));
    targetCellIndex = find(cumulativeProbabilities >= rand(), 1);
    [targetRow, targetCol] = ind2sub([gridSize(1), gridSize(2)], targetCellIndex);
    targetPosition = [targetRow-1, targetCol-1];
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