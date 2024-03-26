close all;

gridSize = [100, 100];
createFigure = true;
numIterations = 10;
targetFactor = 5;
terrainNum = 10;

% for t = 1:terrainNum
%     [featureVertices, probabilityGrid] = getScene(t, gridSize, createFigure, true);
%     targetLocationCounts = zeros(gridSize);
%     for i = 1:numIterations
%         roi = getTarget(probabilityGrid, targetFactor);
%         targetLocationCounts(roi(3), roi(1)) = targetLocationCounts(roi(3), roi(1)) + 1;
%     end
% 
%     figure(t);
%     imagesc(targetLocationCounts);
%     colorbar;
%     axis equal;
%     title('Target Placements Heatmap');
%     xlim([1 gridSize(1)]); xlabel('x (m)');
%     ylim([1 gridSize(2)]); ylabel('y (m)');
%     set(gca, 'YDir', 'normal');
% end

targetFactors = [0, 0.5, 1.0, 2.0, 5, 10];
[featureVertices, probabilityGrid] = getScene(8, gridSize, createFigure, true);
for f = 1:length(targetFactors)
    targetLocationCounts = zeros(gridSize);
    for i = 1:numIterations
        roi = getTarget(probabilityGrid, targetFactors(f));
        targetLocationCounts(roi(3), roi(1)) = targetLocationCounts(roi(3), roi(1)) + 1;
    end

    figure(f);
    imagesc(targetLocationCounts);
    colorbar;
    axis equal;
    title('Target Placements Heatmap');
    xlim([1 gridSize(1)]); xlabel('x (m)');
    ylim([1 gridSize(2)]); ylabel('y (m)');
    set(gca, 'YDir', 'normal');
end

% [featureVertices, probabilityGrid] = getScene(8, gridSize, createFigure, true);
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