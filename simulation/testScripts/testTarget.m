close all;
% terrainNum = 2;
gridSize = [100, 100];
createFigure = false;
numIterations = 20000;

for terrainNum = 1:10
    [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure);
    targetLocationCounts = zeros(gridSize);
    for i = 1:numIterations
        roi = getTarget(probabilityGrid);
        targetLocationCounts(roi(1), roi(3)) = targetLocationCounts(roi(1), roi(3)) + 1;
    end

    figure(terrainNum);
    imagesc(targetLocationCounts);
    colorbar;
    axis equal;
    title('Target Placements Heatmap');
    xlim([1 gridSize(1)]); xlabel('x (m)');
    ylim([1 gridSize(2)]); ylabel('y (m)');
    set(gca, 'YDir', 'normal');
end