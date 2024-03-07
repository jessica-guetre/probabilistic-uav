maxWaypoints = max(cellfun(@(c) numel(c), allWaypointsData)) / 3;
paddedWaypointsData = zeros(size(allWaypointsData, 1), maxWaypoints * 2);

maxWaypoints = max(cellfun(@(c) numel(c), allWaypointsData));
paddedWaypointsMatrix = zeros(length(allWaypointsData), maxWaypoints);
numWaypoints = maxWaypoints / 3;

for i = 1:size(allWaypointsData, 1)
    currentWaypoints = allWaypointsData{i};
    for j = 1:length(currentWaypoints)/3
        paddedWaypointsData(i, (j-1)*2+1:j*2) = currentWaypoints((j-1)*3+1:(j-1)*3+2); % Extract X, Y and ignore Z
    end
end

targetData = paddedWaypointsData;
inputData = {allInputTerrain, allInputPosition};

terrainLayers = [
    imageInputLayer([100, 100, 1], 'Normalization', 'none', 'Name', 'terrain_input')
    convolution2dLayer(3, 8, 'Padding', 'same', 'Name', 'conv1')
    reluLayer('Name', 'relu1')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'maxpool1')
    convolution2dLayer(3, 16, 'Padding', 'same', 'Name', 'conv2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(64, 'Name', 'fc1_terrain')];

positionLayers = [
    featureInputLayer(3, 'Name', 'position_input')
    fullyConnectedLayer(50, 'Name', 'fc_position')
    reluLayer('Name', 'relu_position')];

flattenTerrain = flattenLayer('Name', 'flatten_terrain');
concatLayer = concatenationLayer(1, 2, 'Name', 'concat');

fullyConnectedSize = 64 + 50;
combineLayers = [
    flattenLayer('Name', 'flatten_terrain')
    concatLayer
    reluLayer('Name', 'relu_combine')
    fullyConnectedLayer(fullyConnectedSize, 'Name', 'fc_combine')
    fullyConnectedLayer(maxWaypoints * 2, 'Name', 'fc_final')
    regressionLayer('Name', 'output')
];

lgraph = layerGraph();
lgraph = addLayers(lgraph, terrainLayers);
lgraph = addLayers(lgraph, positionLayers);
lgraph = addLayers(lgraph, combineLayers);
lgraph = connectLayers(lgraph, 'fc1_terrain', 'flatten_terrain');
lgraph = connectLayers(lgraph, 'relu_position', 'concat/in2');

layers = lgraph.Layers;
connections = lgraph.Connections;
net = createLgraphUsingConnections(layers, connections);

options = trainingOptions('adam', ...
    'MaxEpochs', 1, ... % TODO: CHANGE BACK TO 100
    'MiniBatchSize', 16, ...
    'InitialLearnRate', 0.01, ...
    'GradientThreshold', 1, ...
    'Shuffle', 'every-epoch', ...
    'Verbose', false, ...
    'Plots', 'training-progress');

% Reformat? inputData and targetData should be stored in a single variable
[trainedNet, info] = trainNetwork(inputData, targetData, net, options);
