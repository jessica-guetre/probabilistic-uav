updateRate = 3; % in Hz
simTime = 1000;
gridSize = [100, 100];
xlimitsScene = [0 100];
ylimitsScene = [0 100];
zlimitsScene = [-5 40];

% Create the UAV scenario.
gridScene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200);
gridScene.addInertialFrame("ENU","MAP",trvec2tform([1 0 0])); 
addMesh(gridScene,"polygon",{[0 0; gridSize(1) 0; gridSize(1) gridSize(2); 0 gridSize(2)],[-5 0]},0.651*ones(1,3));

% Define terrain features
terrainFeatures = struct('Path', 1, 'River', 2, 'Tree', 3, 'Steep', 4, 'Empty', 5);
terrainFeatureNames = {'Path', 'River', 'Tree', 'Steep', 'Empty'};

% Assign weights and colors to each terrain feature
weights = struct('Path', 5, 'River', 1, 'Tree', 2, 'Steep', 1, 'Empty', 3);
colors = struct('Path', [1 0 0], 'River', [0 0 1], 'Tree', [0 1 0], 'Steep', [0 0 0], 'Empty', [1 1 1], 'Target', [0.4940, 0.1840, 0.5560]);
weightedGrid = weights.Empty * ones(gridSize);

% Populate the grid with terrain features
featureVertices = struct();
featureVertices.Path = {[1, 30; gridSize(1), 30; gridSize(1), 31; 1, 31]};
featureVertices.River = {[40, 1; 45, 1; 45, gridSize(2); 40, gridSize(2)]};
featureVertices.Tree = {
    [20, 1; gridSize(1), 1; gridSize(1), 10; 20, 10], % Tree1
    [60, 10; gridSize(1), 10; gridSize(1), 20; 60, 20] % Tree2
};
featureVertices.Steep = {
    [1, 98; gridSize(1), 98; gridSize(1), gridSize(2); 1, gridSize(2)], % Steep1
    [10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98] % Steep2
};

% Add a mesh for each terrain feature based on the defined vertices
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

% Set up the 3D view of the scenario.
[ax, plotFrames] = show3D(gridScene);
xlim(xlimitsScene);
ylim(ylimitsScene);
zlim(zlimitsScene);