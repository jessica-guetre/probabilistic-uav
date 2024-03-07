function featureVertices = getFeatureVertices(terrainType, gridSize)
    featureVertices = struct();

    switch terrainType
        case 'Type1'
            featureVertices.Path = {[1, 30; gridSize(1), 30; gridSize(1), 35; 1, 35]};
            featureVertices.Water = {
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
            featureVertices.Water = {[1, 50; gridSize(1), 50; gridSize(1), 60; 1, 60]};
            featureVertices.Tree = {
                [20, 1; gridSize(1), 1; gridSize(1), 20; 20, 20],
                [20, 25; gridSize(1), 25; gridSize(1), 45; 20, 45]
            };
            featureVertices.Steep = {[10, 90; gridSize(1), 90; gridSize(1), 98; 10, 98]};
        case 'Type3'
            featureVertices.Path = {[10, 1; 20, 1; 20, gridSize(2); 10, gridSize(2)]};
            featureVertices.Water = {[50, 1; 60, 1; 60, 30; 80, 30; 80, 70; 70, 70; 70, gridSize(2); 50, gridSize(2)]};
            featureVertices.Tree = {[60, 1; gridSize(1), 1; gridSize(1), 30; 60, 30]};
            featureVertices.Steep = {[70, 70; gridSize(1), 70; gridSize(1), gridSize(2); 70, gridSize(2)]};
        case 'Type4'
            featureVertices.Path = {[10, 0; 15, 0; 15, gridSize(2); 10, gridSize(2)]};
            featureVertices.Water = {
                [30, 20; 35, 20; 35, gridSize(2); 30, gridSize(2)]
                [30, 20; gridSize(1), 20; gridSize(1), 30; 30, 30]
                };
            featureVertices.Tree = {
                [50, 50; gridSize(1) - 30, 50; gridSize(1) - 30, gridSize(2) - 20; 50, gridSize(2) - 20],
                [40, 0; gridSize(1), 0; gridSize(1), 20; 40, 20]
                [0, gridSize(2) - 20; 10, gridSize(2) - 20; 10, gridSize(2); 0, gridSize(2)]
            };
            featureVertices.Steep = {[55, gridSize(2) - 20; 85, gridSize(2) - 20; 85, gridSize(2) - 5; 55, gridSize(2) - 5]};
        case 'Type5'
            featureVertices.Path = {
                [0, gridSize(2) - 20; gridSize(1) - 40, gridSize(2) - 20; gridSize(1) - 40, gridSize(2) - 35; 0, gridSize(2) - 35],
                [gridSize(1) - 40, gridSize(2) - 35; gridSize(1) - 20, gridSize(2) - 35; gridSize(1) - 20, gridSize(2); gridSize(1) - 40, gridSize(2)]
                };
            featureVertices.Water = {
                [0, 0; 45, 0; 45, 20; 0, 20]
                [gridSize(1) - 10, 15; gridSize(1), 15; gridSize(1), gridSize(2); gridSize(1) - 10, gridSize(2)]
            };
            featureVertices.Tree = {
                [45, 0; gridSize(1) - 40, 0; gridSize(1) - 40, 40; 45, 40],
                [0, 20; 45, 20; 45, 40; 0, 40]
            };
            featureVertices.Steep = {[gridSize(1) - 40, 0; gridSize(1) - 20, 0; gridSize(1) - 20, 10; gridSize(1) - 40, 10]};
    end
end