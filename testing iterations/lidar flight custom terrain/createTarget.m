function [scene, roi] = createTarget(scene, xbounds, ybounds, zlimits, size, xinitial, yiniital)
    % createScene - ...
    %
    %   [scene, roi] = createTarget(scene, xbounds, ybounds, zlimits, size, updateRate, simTime)
    %
    % Inputs:
    %   - xbounds: ...
    %   - ybounds: ...
    %   - zlimits: ...
    %   - updateRate: ...
    %   - simTime: ...
    %
    % Outputs:
    %   - scene: UAV scenario object with polygon mesh.
    %   - roi: ...
    margin = 2;

    % Randomly generate x and y location from bounds.
    x = (xbounds(2)-xbounds(1))*rand + xbounds(1);
    y = (ybounds(2)-ybounds(1))*rand + ybounds(1);

    % Create x and y limits given target size.
    xlimits = [x x+size];
    ylimits = [y y+size];

    % Define vertices and add polygon to scene.
    vertices = [xlimits(1) ylimits(1); xlimits(2) ylimits(1); xlimits(2) ylimits(2); xlimits(1) ylimits(2)];
    addMesh(scene, "polygon", {vertices, zlimits}, [1 0 0]);

    % Define ROI.
    roi = [(xlimits(1)-margin) (xlimits(2)+margin), (ylimits(1)-margin) (ylimits(2)+margin), (zlimits(1)-margin) (zlimits(2)+margin)];
end
