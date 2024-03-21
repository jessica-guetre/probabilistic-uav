close all;

gridSize = [100, 100];
createFigure = true;
numIterations = 500;
targetFactor = 5;
terrainNum = 10;

for t = 1:terrainNum
    [featureVertices, probabilityGrid] = getScene(t, gridSize, createFigure);
    targetLocationCounts = zeros(gridSize);
    for i = 1:numIterations
        roi = getTarget(probabilityGrid, targetFactor);
        targetLocationCounts(roi(3), roi(1)) = targetLocationCounts(roi(3), roi(1)) + 1;
    end

    figure(t);
    imagesc(targetLocationCounts);
    colorbar;
    axis equal;
    title('Target Placements Heatmap');
    xlim([1 gridSize(1)]); xlabel('x (m)');
    ylim([1 gridSize(2)]); ylabel('y (m)');
    set(gca, 'YDir', 'normal');
end

% [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure);
% targetLocationCounts = zeros(gridSize);
% for i = 1:numIterations
%     roi = getTarget(probabilityGrid, targetFactor);
%     targetLocationCounts(roi(1), roi(3)) = targetLocationCounts(roi(1), roi(3)) + 1;
% end
%
% figure(terrainNum);
% imagesc(targetLocationCounts);
% colorbar;
% axis equal;
% title('Target Placements Heatmap');
% set(gca, 'YDir', 'normal');
% xlim([1 gridSize(2)]); xlabel('x (m)');
% ylim([1 gridSize(1)]); ylabel('y (m)');