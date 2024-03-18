close all;
terrainNum = 2;
gridSize = [500, 500];
createFigure = true;

row = 10;
col = 10;
targetRoi = [row, row + 1, col, col + 1, 0, 3];

for terrainNum = 1:10
    [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure, targetRoi);
    % [featureVertices, probabilityGrid] = getScene(selectedTerrainType, gridSize, createFigure);
end