function featureVertices = getFeatureVertices(terrainType, gridSize)
    featureVertices = struct();

    switch terrainType
        case 'Type1'
            featureVertices.Path = {[1, 30; gridSize(1), 30; gridSize(1), 35; 1, 35]};
            featureVertices.River = {
                [40, 1; 45, 1; 45, 30; 40, 30],
                [40, 35; 45, 35; 45, 90; 40, 90]
            };
            featureVertices.Tree = {
                [20, 1; 40, 1; 40, 10; 20, 10],
                [45, 1; gridSize(1), 1; gridSize(1), 10; 45, 10],
                [60, 10; gridSize(1), 10; gridSize(1), 20; 60, 20]
            };
            featureVertices.Steep = {
                [1, 98; gridSize(1), 98; gridSize(1), gridSize(2); 1, gridSize(2)],
                [10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]
            };
        case 'Type2'
            featureVertices.Path = {[1, 20; gridSize(1), 20; gridSize(1), 25; 1, 25]};
            featureVertices.River = {[1, 50; gridSize(1), 50; gridSize(1), 60; 1, 60]};
            featureVertices.Tree = {
                [20, 1; gridSize(1), 1; gridSize(1), 20; 20, 20],
                [20, 25; gridSize(1), 25; gridSize(1), 45; 20, 45]
            };
            featureVertices.Steep = {[10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]};
        case 'Type3'
            featureVertices.Path = {[10, 1; 20, 1; 20, gridSize(2); 10, gridSize(2)]};
            featureVertices.River = {[50, 1; 60, 1; 60, 30; 80, 30; 80, 70; 70, 70; 70, gridSize(2); 50, gridSize(2)]};
            featureVertices.Tree = {[60, 1; gridSize(1), 1; gridSize(1), 30; 60, 30]};
            featureVertices.Steep = {[70, 70; gridSize(1), 70; gridSize(1), gridSize(2); 70, gridSize(2)]};
    end
end