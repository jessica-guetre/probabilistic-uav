function featureVertices = getFeatureVertices(terrainType, gridSize)
    featureVertices = struct();

    scaleFactorX = gridSize(1) / 100;
    scaleFactorY = gridSize(2) / 100;

    switch terrainType
        case 'Type1'
            featureVertices.Trail = {[0, 30*scaleFactorY; gridSize(1), 30*scaleFactorY; gridSize(1), 35*scaleFactorY; 0, 35*scaleFactorY]};
            featureVertices.Water = {
                [40*scaleFactorX, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 30*scaleFactorY; 40*scaleFactorX, 30*scaleFactorY],
                [40*scaleFactorX, 35*scaleFactorY; 45*scaleFactorX, 35*scaleFactorY; 45*scaleFactorX, 90*scaleFactorY; 40*scaleFactorX, 90*scaleFactorY]
            };
            featureVertices.Forest = {
                [20*scaleFactorX, 0; 40*scaleFactorX, 0; 40*scaleFactorX, 10*scaleFactorY; 20*scaleFactorX, 10*scaleFactorY],
                [45*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 10*scaleFactorY; 45*scaleFactorX, 10*scaleFactorY],
                [60*scaleFactorX, 10*scaleFactorY; gridSize(1), 10*scaleFactorY; gridSize(1), 20*scaleFactorY; 60*scaleFactorX, 20*scaleFactorY]
            };
            featureVertices.Elevation = {
                [0, 98*scaleFactorY; gridSize(1), 98*scaleFactorY; gridSize(1), gridSize(2); 0, gridSize(2)],
                [10*scaleFactorX, 90*scaleFactorY; gridSize(1), 90*scaleFactorY; gridSize(1), 98*scaleFactorY; 10*scaleFactorX, 98*scaleFactorY]
            };
        case 'Type2'
            featureVertices.Trail = {[0, 20*scaleFactorY; gridSize(1), 20*scaleFactorY; gridSize(1), 25*scaleFactorY; 0, 25*scaleFactorY]};
            featureVertices.Water = {[0, 50*scaleFactorY; gridSize(1), 50*scaleFactorY; gridSize(1), 60*scaleFactorY; 0, 60*scaleFactorY]};
            featureVertices.Forest = {
                [20*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 20*scaleFactorY; 20*scaleFactorX, 20*scaleFactorY],
                [20*scaleFactorX, 25*scaleFactorY; gridSize(1), 25*scaleFactorY; gridSize(1), 45*scaleFactorY; 20*scaleFactorX, 45*scaleFactorY]
            };
            featureVertices.Elevation = {[10*scaleFactorX, 90*scaleFactorY; gridSize(1), 90*scaleFactorY; gridSize(1), 98*scaleFactorY; 10*scaleFactorX, 98*scaleFactorY]};
        case 'Type3'
            featureVertices.Trail = {[10*scaleFactorX, 0; 20*scaleFactorX, 0; 20*scaleFactorX, gridSize(2); 10*scaleFactorX, gridSize(2)]};
            featureVertices.Water = {
                [50*scaleFactorX, 0; 60*scaleFactorX, 0; 60*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 30*scaleFactorY; 80*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, 70*scaleFactorY; 70*scaleFactorX, gridSize(2); 50*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {[60*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 30*scaleFactorY; 60*scaleFactorX, 30*scaleFactorY]};
            featureVertices.Elevation = {[70*scaleFactorX, 70*scaleFactorY; gridSize(1), 70*scaleFactorY; gridSize(1), gridSize(2); 70*scaleFactorX, gridSize(2)]};
        case 'Type4'
            featureVertices.Trail = {[10*scaleFactorX, 0; 15*scaleFactorX, 0; 15*scaleFactorX, gridSize(2); 10*scaleFactorX, gridSize(2)]};
            featureVertices.Water = {
                [30*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, 20*scaleFactorY; 35*scaleFactorX, gridSize(2); 30*scaleFactorX, gridSize(2)],
                [30*scaleFactorX, 20*scaleFactorY; gridSize(1), 20*scaleFactorY; gridSize(1), 30*scaleFactorY; 30*scaleFactorX, 30*scaleFactorY]
            };
            featureVertices.Forest = {
                [50*scaleFactorX, 50*scaleFactorY; gridSize(1) - 30*scaleFactorX, 50*scaleFactorY; gridSize(1) - 30*scaleFactorX, gridSize(2) - 20*scaleFactorY; 50*scaleFactorX, gridSize(2) - 20*scaleFactorY],
                [40*scaleFactorX, 0; gridSize(1), 0; gridSize(1), 20*scaleFactorY; 40*scaleFactorX, 20*scaleFactorY],
                [0, gridSize(2) - 20*scaleFactorY; 10*scaleFactorX, gridSize(2) - 20*scaleFactorY; 10*scaleFactorX, gridSize(2); 0, gridSize(2)]
            };
            featureVertices.Elevation = {[55*scaleFactorX, gridSize(2) - 20*scaleFactorY; 85*scaleFactorX, gridSize(2) - 20*scaleFactorY; 85*scaleFactorX, gridSize(2) - 5*scaleFactorY; 55*scaleFactorX, gridSize(2) - 5*scaleFactorY]};
        case 'Type5'
            featureVertices.Trail = {
                [0, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; 0, gridSize(2) - 35*scaleFactorY],
                [gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2); gridSize(1) - 40*scaleFactorX, gridSize(2)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(1) - 10*scaleFactorX, 15*scaleFactorY; gridSize(1), 15*scaleFactorY; gridSize(1), gridSize(2); gridSize(1) - 10*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 10*scaleFactorY; gridSize(1) - 40*scaleFactorX, 10*scaleFactorY]};
        case 'Type6'
            featureVertices.Trail = {
                [0, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; 0, gridSize(2) - 35*scaleFactorY],
                [gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2); gridSize(1) - 40*scaleFactorX, gridSize(2)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(1) - 10*scaleFactorX, 15*scaleFactorY; gridSize(1), 15*scaleFactorY; gridSize(1), gridSize(2); gridSize(1) - 10*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 10*scaleFactorY; gridSize(1) - 40*scaleFactorX, 10*scaleFactorY]};
        case 'Type7'
            featureVertices.Trail = {
                [0, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; 0, gridSize(2) - 35*scaleFactorY],
                [gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2); gridSize(1) - 40*scaleFactorX, gridSize(2)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(1) - 10*scaleFactorX, 15*scaleFactorY; gridSize(1), 15*scaleFactorY; gridSize(1), gridSize(2); gridSize(1) - 10*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 10*scaleFactorY; gridSize(1) - 40*scaleFactorX, 10*scaleFactorY]};
        case 'Type8'
            featureVertices.Trail = {
                [0, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; 0, gridSize(2) - 35*scaleFactorY],
                [gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2); gridSize(1) - 40*scaleFactorX, gridSize(2)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(1) - 10*scaleFactorX, 15*scaleFactorY; gridSize(1), 15*scaleFactorY; gridSize(1), gridSize(2); gridSize(1) - 10*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 10*scaleFactorY; gridSize(1) - 40*scaleFactorX, 10*scaleFactorY]};
        case 'Type9'
            featureVertices.Trail = {
                [0, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; 0, gridSize(2) - 35*scaleFactorY],
                [gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2); gridSize(1) - 40*scaleFactorX, gridSize(2)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(1) - 10*scaleFactorX, 15*scaleFactorY; gridSize(1), 15*scaleFactorY; gridSize(1), gridSize(2); gridSize(1) - 10*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 10*scaleFactorY; gridSize(1) - 40*scaleFactorX, 10*scaleFactorY]};
        case 'Type10'
            featureVertices.Trail = {
                [0, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 20*scaleFactorY; gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; 0, gridSize(2) - 35*scaleFactorY],
                [gridSize(1) - 40*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2) - 35*scaleFactorY; gridSize(1) - 20*scaleFactorX, gridSize(2); gridSize(1) - 40*scaleFactorX, gridSize(2)]
            };
            featureVertices.Water = {
                [0, 0; 45*scaleFactorX, 0; 45*scaleFactorX, 20*scaleFactorY; 0, 20*scaleFactorY],
                [gridSize(1) - 10*scaleFactorX, 15*scaleFactorY; gridSize(1), 15*scaleFactorY; gridSize(1), gridSize(2); gridSize(1) - 10*scaleFactorX, gridSize(2)]
            };
            featureVertices.Forest = {
                [45*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 40*scaleFactorX, 40*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY],
                [0, 20*scaleFactorY; 45*scaleFactorX, 20*scaleFactorY; 45*scaleFactorX, 40*scaleFactorY; 0, 40*scaleFactorY]
            };
            featureVertices.Elevation = {[gridSize(1) - 40*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 0; gridSize(1) - 20*scaleFactorX, 10*scaleFactorY; gridSize(1) - 40*scaleFactorX, 10*scaleFactorY]};
    end
end