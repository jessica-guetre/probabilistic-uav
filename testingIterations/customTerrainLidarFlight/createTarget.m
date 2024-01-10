function [scene, roi] = createTarget(scene, xbounds, ybounds, zlimits, size, initialPosition)
    % createTarget - Adds a target to a UAV scenario scene.
    %
    % This function adds a polygon mesh representing a target to a given 
    % UAV scenario scene based on provided spatial bounds and size. If an 
    % initial position is provided, the target is placed there; otherwise, 
    % a random position within bounds is chosen.
    %
    % Syntax:
    %   [scene, roi] = createTarget(scene, xbounds, ybounds, zlimits, size, initialPosition)
    %
    % Inputs:
    %   - scene: UAV scenario object.
    %   - xbounds: 1x2 array, [minX, maxX], defining the X-coordinate bounds.
    %   - ybounds: 1x2 array, [minY, maxY], defining the Y-coordinate bounds.
    %   - zlimits: 1x2 array, [minZ, maxZ], defining the Z-coordinate limits.
    %   - size: Scalar, size of the target.
    %   - initialPosition: Optional 1x2 array, [xinitial, yinitial], specifying the initial position.
    %
    % Outputs:
    %   - scene: UAV scenario object with the added polygon mesh.
    %   - roi: Region of interest defined around the target.
    margin = 2;

    % Check if initialPosition is provided and non-empty.
    if nargin < 6 || isempty(initialPosition)
        % Randomly generate x and y location from bounds.
        x = (xbounds(2)-xbounds(1))*rand + xbounds(1);
        y = (ybounds(2)-ybounds(1))*rand + ybounds(1);
    else
        % Use the provided initial position.
        x = initialPosition(1);
        y = initialPosition(2);
    end

    % Create x and y limits given target size.
    xlimits = [x x+size];
    ylimits = [y y+size];

    % Define vertices and add polygon to scene.
    vertices = [xlimits(1) ylimits(1); xlimits(2) ylimits(1); xlimits(2) ylimits(2); xlimits(1) ylimits(2)];
    addMesh(scene, "polygon", {vertices, zlimits}, [1 0 0]);

    % Define ROI.
    roi = [(xlimits(1)-margin) (xlimits(2)+margin), (ylimits(1)-margin) (ylimits(2)+margin), (zlimits(1)-margin) (zlimits(2)+margin)];
end
