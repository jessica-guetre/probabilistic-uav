function [featureVertices, probabilityGrid] = getScene(terrainNum, gridSize, createFigure, forTarget, targetRoi)
% Constructs  scene by defining feature vertices and calculating probability grids based on terrain features.
%
% Inputs:
% - terrainNum (scalar): Index specifying the terrain type.
% - gridSize (2-element vector): Size of the grid [rows, cols].
% - createFigure (boolean): Indicates whether to create visualizations.
% - forTarget (boolean): Adjusts probability calculations if the scene is for target placement.
% - targetRoi (4-element vector, optional): Specifies the region of interest for the target [startRow, endRow, startCol, endCol].
%
% Outputs:
% - featureVertices (struct): Contains vertices for each terrain feature.
% - probabilityGrid (matrix): Grid of probabilities for each cell, representing the likelihood of finding the target.

    terrainTypes = {'Type1', 'Type2', 'Type3', 'Type4', 'Type5', 'Type6', 'Type7', 'Type8', 'Type9', 'Type10'};
    colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667], 'Field', [0.4667 0.6745 0.1882], 'Target', [1 0 0]);
    probabilities = struct('Trail', [-1, 7.0; 0, 7.0; 50, 2.6; 100, 1.7; 150, 1.4; 200, 1.3], 'Water', [-1, 0.5; 0, 5.2; 50, 4.5; 100, 3.8; 150, 3.1; 200, 2.6], 'Forest', [-1, 0.7], 'Elevation', [-1, 1.8]);

    featureVertices = getFeatureVertices(terrainTypes{terrainNum}, gridSize);
    probabilityGrid = terrainProbabilities(featureVertices, gridSize, probabilities, forTarget);

    if createFigure
        figure(terrainNum * 10); % feature probability heatmap
        imagesc(probabilityGrid);
        colorbar;
        axis equal;
        title('Feature Probability Heatmap');
        set(gca, 'YDir', 'normal');
        xlim([1 gridSize(2)]); xlabel('x (m)');
        ylim([1 gridSize(1)]); ylabel('y (m)');

        figure(terrainNum * 10 + 1); % feature map
        hold on;
        axis equal;
        title('Feature Map');
        set(gca, 'YDir', 'normal');
        xlim([1 gridSize(2)]); xlabel('x (m)');
        ylim([1 gridSize(1)]); ylabel('y (m)');
        title('Map of Features');
        patch('Vertices', [1, 1; gridSize(2), 1; gridSize(2), gridSize(1); 1, gridSize(1)], 'Faces', [1, 2, 3, 4], 'FaceColor', colors.('Field'), 'EdgeColor', 'none');
        legendEntries = {'Field'};
        colorsForLegend = colors.('Field');
        for featureType = fieldnames(featureVertices)'
            featureName = featureType{1};
            featureArray = featureVertices.(featureName);
            color = colors.(featureName);
            for i = 1:length(featureArray)
                vertices = featureArray{i};
                patch('Vertices', vertices, 'Faces', 1:size(vertices, 1), 'FaceColor', color, 'EdgeColor', 'none');
            end
            legendEntries{end+1} = featureName;
            colorsForLegend(end+1, :) = color;
        end

        if nargin == 5
            targetVertices = [targetRoi(3), targetRoi(1); targetRoi(4), targetRoi(1); targetRoi(4), targetRoi(2); targetRoi(3), targetRoi(2)];
            patch('Vertices', targetVertices, 'Faces', [1, 2, 3, 4], 'FaceColor', colors.Target, 'EdgeColor', 'none');
            legendEntries{end + 1} = 'Target';
            colorsForLegend = [colorsForLegend; colors.Target];
        end

        for i = 1:length(legendEntries)
            h(i) = patch(NaN, NaN, colorsForLegend(i,:), 'EdgeColor', 'none', 'DisplayName', legendEntries{i});
        end
        legend(h);
        hold off;
    end
end

function featureVertices = getFeatureVertices(terrainType, gridSize)
% Generates feature vertices for a specified terrain type within a given grid size.
%
% Inputs:
% - terrainType (string): Specifies the type of terrain to generate features for.
% - gridSize (2-element vector): Size of the grid [rows, cols] for scaling the features.
%
% Output:
% - featureVertices (struct): Contains vertices for each terrain feature, scaled according to the grid size.

    featureVertices = struct();

    scaleFactorX = gridSize(2) / 100;
    scaleFactorY = gridSize(1) / 100;

    switch terrainType
        case 'Type1'
            featureVertices.Trail = {[0, 30*scaleFactorY; gridSize(2), 30*scaleFactorY; gridSize(2), 35*scaleFactorY; 0, 35*scaleFactorY]};
            featureVertices.Water = {
                [40*scaleFactorX, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 30*scaleFactorY; 40*scaleFactorX, 30*scaleFactorY],
                [40*scaleFactorX, 35*scaleFactorY; 45*scaleFactorX, 35*scaleFactorY; 45*scaleFactorX, 90*scaleFactorY; 40*scaleFactorX, 90*scaleFactorY]
            };
            featureVertices.Forest = {
                [20*scaleFactorX, 0; 40*scaleFactorX, 0; 40*scaleFactorX, 10*scaleFactorY; 20*scaleFactorX, 10*scaleFactorY],
                [45*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 10*scaleFactorY; 45*scaleFactorX, 10*scaleFactorY],
                [60*scaleFactorX, 10*scaleFactorY; gridSize(2), 10*scaleFactorY; gridSize(2), 20*scaleFactorY; 60*scaleFactorX, 20*scaleFactorY]
            };
            featureVertices.Elevation = {
                [0, 98*scaleFactorY; gridSize(2), 98*scaleFactorY; gridSize(2), gridSize(1); 0, gridSize(1)],
                [10*scaleFactorX, 90*scaleFactorY; gridSize(2), 90*scaleFactorY; gridSize(2), 98*scaleFactorY; 10*scaleFactorX, 98*scaleFactorY]
            };
        case 'Type2'
            featureVertices.Trail = {[0, 20*scaleFactorY; gridSize(2), 20*scaleFactorY; gridSize(2), 25*scaleFactorY; 0, 25*scaleFactorY]};
            featureVertices.Water = {[0, 50*scaleFactorY; gridSize(2), 50*scaleFactorY; gridSize(2), 60*scaleFactorY; 0, 60*scaleFactorY]};
            featureVertices.Forest = {
                [20*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 20*scaleFactorY; 20*scaleFactorX, 20*scaleFactorY],
                [20*scaleFactorX, 25*scaleFactorY; gridSize(2), 25*scaleFactorY; gridSize(2), 45*scaleFactorY; 20*scaleFactorX, 45*scaleFactorY]
            };
            featureVertices.Elevation = {[10*scaleFactorX, 90*scaleFactorY; gridSize(2), 90*scaleFactorY; gridSize(2), 98*scaleFactorY; 10*scaleFactorX, 98*scaleFactorY]};
        case 'Type3'
            featureVertices.Trail = {[10*scaleFactorX, 0; 20*scaleFactorX, 0; 20*scaleFactorX, gridSize(1); 10*scaleFactorX, gridSize(1)]};
            featureVertices.Water = {[50*scaleFactorX, 0; 60*scaleFactorX, 0; 60*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, gridSize(1); 50*scaleFactorX, gridSize(1)]};
            featureVertices.Forest = {[60*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 30*scaleFactorY; 60*scaleFactorX, 30*scaleFactorY]};
            featureVertices.Elevation = {[70*scaleFactorX, 70*scaleFactorY; gridSize(2), 70*scaleFactorY; gridSize(2), gridSize(1); 70*scaleFactorX, gridSize(1)]};
        case 'Type4'
            featureVertices.Trail = {[10*scaleFactorX, 0; 15*scaleFactorX, 0; 15*scaleFactorX, gridSize(1); 10*scaleFactorX, gridSize(1)]};
            featureVertices.Water = {
                [30*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, gridSize(1); 30*scaleFactorX, gridSize(1)],
                [30*scaleFactorX, 20*scaleFactorY; gridSize(2), 20*scaleFactorY; gridSize(2), 30*scaleFactorY; 30*scaleFactorX, 30*scaleFactorY]
            };
            featureVertices.Forest = {
                [50*scaleFactorX, 50*scaleFactorY; gridSize(2) - 30*scaleFactorX, 50*scaleFactorY; gridSize(2) - 30*scaleFactorX, gridSize(1) - 20*scaleFactorY; 50*scaleFactorX, gridSize(1) - 20*scaleFactorY],
                [40*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 20*scaleFactorY; 40*scaleFactorX, 20*scaleFactorY],
                [0, gridSize(1) - 20*scaleFactorY; 10*scaleFactorX, gridSize(1) - 20*scaleFactorY; 10*scaleFactorX, gridSize(1); 0, gridSize(1)]
            };
            featureVertices.Elevation = {[55*scaleFactorX, gridSize(1) - 20*scaleFactorY; 85*scaleFactorX, gridSize(1) - 20*scaleFactorY; 85*scaleFactorX, gridSize(1) - 5*scaleFactorY; 55*scaleFactorX, gridSize(1) - 5*scaleFactorY]};
        case 'Type5'
            featureVertices.Trail = {
                [0, gridSize(1) - 20*scaleFactorY; gridSize(2) - 40*scaleFactorX, gridSize(1) - 20*scaleFactorY; gridSize(2) - 40*scaleFactorX, gridSize(1) - 35*scaleFactorY; 0, gridSize(1) - 35*scaleFactorY],
                [gridSize(2) - 40*scaleFactorX, gridSize(1) - 35*scaleFactorY; gridSize(2) - 20*scaleFactorX, gridSize(1) - 35*scaleFactorY; gridSize(2) - 20*scaleFactorX, gridSize(1); gridSize(2) - 40*scaleFactorX, gridSize(1)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(2) - 10*scaleFactorX, 15*scaleFactorY; gridSize(2), 15*scaleFactorY; gridSize(2), gridSize(1); gridSize(2) - 10*scaleFactorX, gridSize(1)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(2) - 40*scaleFactorX, 0; gridSize(2) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(2) - 40*scaleFactorX, 0; gridSize(2) - 20*scaleFactorX, 0; gridSize(2) - 20*scaleFactorX, 10*scaleFactorY; gridSize(2) - 40*scaleFactorX, 10*scaleFactorY]};
        case 'Type6'
            featureVertices.Trail = {[0, 20*scaleFactorY; gridSize(2), 20*scaleFactorY; gridSize(2), 25*scaleFactorY; 0, 25*scaleFactorY]};
            featureVertices.Water = {[0, 55*scaleFactorY; gridSize(2), 55*scaleFactorY; gridSize(2), 85*scaleFactorY; 0, 85*scaleFactorY]};
            featureVertices.Forest = {[10*scaleFactorX, 85*scaleFactorY; gridSize(2), 85*scaleFactorY; gridSize(2), 95*scaleFactorY; 10*scaleFactorX, 95*scaleFactorY]};
            featureVertices.Elevation = {
                [20*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 20*scaleFactorY; 20*scaleFactorX, 20*scaleFactorY],
                [20*scaleFactorX, 25*scaleFactorY; gridSize(2), 25*scaleFactorY; gridSize(2), 45*scaleFactorY; 20*scaleFactorX, 45*scaleFactorY]
            };
        case 'Type7'
            featureVertices.Trail = {
                [gridSize(2) - 10*scaleFactorX, 0; gridSize(2), 0; gridSize(2), gridSize(1); gridSize(2) - 10*scaleFactorX, gridSize(1)]
            };
            featureVertices.Water = {
                [45*scaleFactorX, 0; gridSize(2) - 40*scaleFactorX, 0; gridSize(2) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Forest = {
                [0, gridSize(1) - 20*scaleFactorY; gridSize(2) - 40*scaleFactorX, gridSize(1) - 20*scaleFactorY; gridSize(2) - 40*scaleFactorX, gridSize(1) - 35*scaleFactorY; 0, gridSize(1) - 35*scaleFactorY],
                [gridSize(2) - 40*scaleFactorX, gridSize(1) - 35*scaleFactorY; gridSize(2) - 20*scaleFactorX, gridSize(1) - 35*scaleFactorY; gridSize(2) - 20*scaleFactorX, gridSize(1); gridSize(2) - 40*scaleFactorX, gridSize(1)]
            };
            featureVertices.Elevation = {
                [gridSize(2) - 40*scaleFactorX, 0; gridSize(2) - 20*scaleFactorX, 0; gridSize(2) - 20*scaleFactorX, 10*scaleFactorY; gridSize(2) - 40*scaleFactorX, 10*scaleFactorY],
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY]
                };
        case 'Type8'
            featureVertices.Trail = {
                [30*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, gridSize(1); 30*scaleFactorX, gridSize(1)],
                [30*scaleFactorX, 20*scaleFactorY; gridSize(2), 20*scaleFactorY; gridSize(2), 30*scaleFactorY; 30*scaleFactorX, 30*scaleFactorY]
            };
            featureVertices.Water = {
                [10*scaleFactorX, 0; 15*scaleFactorX, 0; 15*scaleFactorX, gridSize(1); 10*scaleFactorX, gridSize(1)],
                [50*scaleFactorX, 50*scaleFactorY; gridSize(2) - 30*scaleFactorX, 50*scaleFactorY; gridSize(2) - 30*scaleFactorX, gridSize(1) - 20*scaleFactorY; 50*scaleFactorX, gridSize(1) - 20*scaleFactorY],

            };
            featureVertices.Forest = {[55*scaleFactorX, gridSize(1) - 20*scaleFactorY; 85*scaleFactorX, gridSize(1) - 20*scaleFactorY; 85*scaleFactorX, gridSize(1) - 5*scaleFactorY; 55*scaleFactorX, gridSize(1) - 5*scaleFactorY]};
            featureVertices.Elevation = {
                [40*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 20*scaleFactorY; 40*scaleFactorX, 20*scaleFactorY],
                [0, gridSize(1) - 20*scaleFactorY; 10*scaleFactorX, gridSize(1) - 20*scaleFactorY; 10*scaleFactorX, gridSize(1); 0, gridSize(1)]
            };
        case 'Type9'
            featureVertices.Trail = {[30*scaleFactorX, 0; 35*scaleFactorX, 0; 35*scaleFactorX, gridSize(1); 30*scaleFactorX, gridSize(1)]};
            featureVertices.Water = {[60*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 30*scaleFactorY; 60*scaleFactorX, 30*scaleFactorY]};
            featureVertices.Forest = {[70*scaleFactorX, 70*scaleFactorY; gridSize(2), 70*scaleFactorY; gridSize(2), gridSize(1); 70*scaleFactorX, gridSize(1)]};
            featureVertices.Elevation = {[50*scaleFactorX, 0; 60*scaleFactorX, 0; 60*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, gridSize(1); 50*scaleFactorX, gridSize(1)]};
        case 'Type10'
            featureVertices.Trail = {[0, 50*scaleFactorY; gridSize(2), 50*scaleFactorY; gridSize(2), 60*scaleFactorY; 0, 60*scaleFactorY]};
            featureVertices.Water = {[0, 20*scaleFactorY; gridSize(2), 20*scaleFactorY; gridSize(2), 25*scaleFactorY; 0, 25*scaleFactorY]};
            featureVertices.Forest = {
                [20*scaleFactorX, 0; gridSize(2), 0; gridSize(2), 20*scaleFactorY; 20*scaleFactorX, 20*scaleFactorY],
                [20*scaleFactorX, 25*scaleFactorY; gridSize(2), 25*scaleFactorY; gridSize(2), 45*scaleFactorY; 20*scaleFactorX, 45*scaleFactorY]
            };
            featureVertices.Elevation = {[10*scaleFactorX, 90*scaleFactorY; gridSize(2), 90*scaleFactorY; gridSize(2), 98*scaleFactorY; 10*scaleFactorX, 98*scaleFactorY]};
    end
end

function probabilityGrid = terrainProbabilities(featureVertices, gridSize, probabilities, forTarget)
% Calculates a probability grid based on terrain features and their respective probabilities.
%
% Inputs:
% - featureVertices (struct): Contains vertices for each terrain feature.
% - gridSize (2-element vector): Size of the grid [rows, cols].
% - probabilities (struct): Probability values associated with each terrain feature.
% - forTarget (boolean): Indicates if the probability adjustments for targeting are applied.
%
% Output:
% - probabilityGrid (matrix): Grid of probabilities for each cell, indicating the likelihood of each terrain feature.

    probabilityGrid = ones(gridSize);
    for featureType = fieldnames(featureVertices)'
        if strcmp(featureType{1}, 'Forest') && forTarget == true % UAV lidar sensor has reduced vision in forest, target does not consider this
            probabilityInfo = [-1, 1.0];
        else
            probabilityInfo = probabilities.(featureType{1});
        end

        binaryMask = zeros(gridSize);
        for j = 1:length(featureVertices.(featureType{1}))
            vertices = featureVertices.(featureType{1}){j};
            mask = poly2mask(vertices(:,1), vertices(:,2), gridSize(1), gridSize(2));
            binaryMask(mask) = 1;
        end
        probabilityGrid(binaryMask == 1) = probabilityInfo(1, 2);
    end
    
    for featureType = fieldnames(featureVertices)'
        probabilityInfo = probabilities.(featureType{1});
    
        binaryMask = zeros(gridSize);
        for j = 1:length(featureVertices.(featureType{1}))
            vertices = featureVertices.(featureType{1}){j};
            mask = poly2mask(vertices(:,1), vertices(:,2), gridSize(1), gridSize(2));
            binaryMask(mask) = 1;
        end
    
        distanceTransform = bwdist(binaryMask);
    
        for k = 2:size(probabilityInfo, 1) - 1
            lowerBoundDistance = probabilityInfo(k, 1);
            upperBoundDistance = probabilityInfo(k+1, 1);
            lowerBoundProbability = probabilityInfo(k, 2);
            upperBoundProbability = probabilityInfo(k+1, 2);
            indices = distanceTransform >= lowerBoundDistance & distanceTransform < upperBoundDistance;
    
            slope = (upperBoundProbability - lowerBoundProbability) / (upperBoundDistance - lowerBoundDistance);
            probabilityGrid(indices) = probabilityGrid(indices) + (lowerBoundProbability + slope * (distanceTransform(indices) - lowerBoundDistance));
        end
    end
    
    probabilityGrid = (probabilityGrid - min(probabilityGrid(:))) / (max(probabilityGrid(:)) - min(probabilityGrid(:)));
end