function [gridScene, probabilityGrid] = terrainSetup(gridScene, gridSize, selectedTerrainType)
    probabilities = struct('Trail', 5, 'Water', 1, 'Forest', 2, 'Elevation', 1, 'Field', 3);
    colors = struct('Trail', [0.4 0.2 0], 'Water', [0.2 0.6 1], 'Forest', [0 0.3333 0], 'Elevation', [0.4667 0.4667 0.4667]);
    probabilityGrid = probabilities.Field * ones(gridSize);
    featureVertices = getFeatureVertices(selectedTerrainType, gridSize);
    for featureType = fieldnames(featureVertices)'
        featureName = featureType{1};
        featureArray = featureVertices.(featureName);

        for i = 1:length(featureArray)
            vertices = featureArray{i};
            mask = poly2mask(vertices(:,1), vertices(:,2), gridSize(1), gridSize(2));
            probabilityGrid(mask) = probabilities.(featureName);
        end
    end

    for featureType = fieldnames(featureVertices)'
        featureArray = featureVertices.(featureType{1});
        for i = 1:length(featureArray)
            vertices = featureArray{i};
            color = colors.(featureType{1});
            addMesh(gridScene, "polygon", {vertices, [0 1]}, color);
        end
    end
end