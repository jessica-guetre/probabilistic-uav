% Define the CNN architecture for terrain input
terrainLayers = [
    imageInputLayer([100, 100, 1], 'Normalization', 'none', 'Name', 'terrain_input')
    convolution2dLayer(3, 8, 'Padding', 'same', 'Name', 'conv1')
    reluLayer('Name', 'relu1')
    maxPooling2dLayer(2, 'Stride', 2, 'Name', 'maxpool1')
    convolution2dLayer(3, 16, 'Padding', 'same', 'Name', 'conv2')
    reluLayer('Name', 'relu2')
    fullyConnectedLayer(64, 'Name', 'fc1_terrain')];

% Define the input layer for UAV's starting position
positionLayers = [
    featureInputLayer(2, 'Name', 'position_input')
    fullyConnectedLayer(50, 'Name', 'fc_position')
    reluLayer('Name', 'relu_position')];

% Combine the CNN and position outputs
combineLayers = [
    additionLayer(2, 'Name', 'add')
    reluLayer('Name', 'relu_combine')
    fullyConnectedLayer(300, 'Name', 'fc_combine') % Adjust based on the expected output size
    regressionLayer('Name', 'output')];

lgraph = layerGraph();
lgraph = addLayers(lgraph, terrainLayers);
lgraph = addLayers(lgraph, positionLayers);
lgraph = addLayers(lgraph, combineLayers);

% Connect the layers
lgraph = connectLayers(lgraph, 'fc1_terrain', 'add/in1');
lgraph = connectLayers(lgraph, 'relu_position', 'add/in2');

% Compile the network architecture
layers = lgraph.Layers;
connections = lgraph.Connections;
net = createLgraphUsingConnections(layers, connections);

% Training options
options = trainingOptions('adam', ...
    'MaxEpochs', 100, ...
    'MiniBatchSize', 16, ...
    'InitialLearnRate', 0.01, ...
    'GradientThreshold', 1, ...
    'Shuffle', 'every-epoch', ...
    'Verbose', false, ...
    'Plots', 'training-progress');

% Train the network
[trainedNet, info] = trainNetwork(inputData, targetData, net, options);
