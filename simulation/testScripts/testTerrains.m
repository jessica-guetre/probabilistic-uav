close all;
gridSize = [600, 1200];
createFigure = true;

row = 10;
col = 10;
targetRoi = [row, row + 1, col, col + 1, 0, 3];

for t = 1:terrainNum
    [featureVertices, probabilityGrid] = getScene(t, gridSize, createFigure, targetRoi);
end