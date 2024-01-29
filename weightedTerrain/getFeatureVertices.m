function featureVertices = getFeatureVertices(terrainType, gridSize)
    featureVertices = struct();

    switch terrainType
        case 'Type1'
            featureVertices.Path = {[1, 30; gridSize(1), 30; gridSize(1), 31; 1, 31]};
            featureVertices.River = {[40, 1; 45, 1; 45, gridSize(2); 40, gridSize(2)]};
            featureVertices.Tree = {
                [20, 1; gridSize(1), 1; gridSize(1), 10; 20, 10],
                [60, 10; gridSize(1), 10; gridSize(1), 20; 60, 20]
            };
            featureVertices.Steep = {
                [1, 98; gridSize(1), 98; gridSize(1), gridSize(2); 1, gridSize(2)],
                [10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]
            };
        case 'Type2'
            featureVertices.Path = {[1, 1; gridSize(1), 1; gridSize(1), 5; 1, 5]};
            featureVertices.River = {[1, 50; gridSize(1), 50; gridSize(1), 51; 1, 51]};
            featureVertices.Tree = {[60, 1; gridSize(1), 1; gridSize(1), 60; 20, 80]};
            featureVertices.Steep = {[10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]};
        case 'Type3'
            featureVertices.Path = {[1, 1; 1, 10; 1, gridSize(2); 10, gridSize(2)]};
            featureVertices.River = {[50, 1; 51, 1; 51, gridSize(2); 50, gridSize(2)]};
            featureVertices.Steep = {[70, 70; 80, 70; 80, 80; 70, 80]};
            featureVertices.Tree = {[60, 20; 70, 20; 70, 30; 60, 30]};
    end
end