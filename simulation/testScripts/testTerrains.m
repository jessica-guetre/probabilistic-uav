close all;
terrainNum = 2;
gridSize = [100, 100];
createFigure = true;

for terrainNum = 1:10
    [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure, [10, 10]);
    % [featureVertices, probabilityGrid] = getScene(selectedTerrainType, gridSize, createFigure);
end

% [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure, [10, 10]);