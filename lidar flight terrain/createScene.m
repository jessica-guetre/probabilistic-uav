function scene = createScene(xlimits, ylimits, zlimits, minHeight, maxHeight, variability, updateRate, simTime)
    % createScene - Create a uavScenario with an uneven surface mesh.
    %
    %   scene = createScene(xlimits, ylimits, zlimits, minHeight, maxHeight, variability, updateRate, simTime)
    %
    % Inputs:
    %   - xlimits: X-axis boundaries of the scenario.
    %   - ylimits: Y-axis boundaries of the scenario.
    %   - zlimits: Z-axis boundaries for the mesh.
    %   - minHeight: Minimum height of the mesh ground.
    %   - maxHeight: Maximum height of the mesh ground.
    %   - variability: Indicates the variability of the ground elevation.
    %   - updateRate: Update rate for the scenario.
    %   - simTime: Simulation time.
    %
    % Outputs:
    %   - scene: UAV scenario object with uneven surface mesh.

    % Create a grid of points within the specified limits.
    [X, Y] = meshgrid(linspace(xlimits(1), xlimits(2), 100), linspace(ylimits(1), ylimits(2), 100));

    % Generate Z values for the grid points with specified variability.
    Z = minHeight + (maxHeight - minHeight) * variability * rand(size(X));

    % Create the UAV scenario.
    scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200);

    % Add the custom mesh to the scene.
    addMesh(scene, X, Y, Z, 'zlimits', zlimits, 'FaceColor', 0.651*ones(1,3));
end
