function [scene, R] = terrainScene(dtedFile, terrainName, terrainColor)
    % terrainScene - Create a UAV scene with custom terrain from a DTED file.
    %
    %   [scene, R] = terrainScene(dtedFile, terrainName, terrainColor)
    %
    % Inputs:
    %   - dtedFile: The DTED file containing terrain data.
    %   - terrainName: Name to assign to the custom terrain.
    %   - terrainColor (optional): Color of the terrain mesh (default: [0.4660 0.6740 0.1880]).
    %
    % Outputs:
    %   - scene: UAV scenario object.
    %   - R: Spatial reference object.
    
    % Check if terrainColor is provided, otherwise, use the default color.
    if nargin < 3
        terrainColor = [0.4660 0.6740 0.1880];
    end
    
    try
        % Read georaster data from the DTED file.
        [A, R] = readgeoraster(dtedFile);
        
        % Determine x and y limits from the raster size.
        xlimits = [1 R.RasterSize(1)];
        ylimits = [1 R.RasterSize(2)];

        % Create a UAV scene with a reference location specified in the center of the terrain.
        scene = uavScenario(ReferenceLocation = [mean(R.LatitudeLimits), mean(R.LongitudeLimits), 0]);

        % Add custom terrain to the scene.
        addCustomTerrain(terrainName, dtedFile);

        % Add the terrain mesh to the scene with the specified color.
        addMesh(scene, "terrain", {terrainName, xlimits, ylimits}, terrainColor);

    catch ME
        % Handle any errors that may occur during file reading or scene creation.
        error('Error creating terrain scene: %s', ME.message);
    end
end
