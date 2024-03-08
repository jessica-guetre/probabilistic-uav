function [scene, plat] = createPlatform(scene, initialVelocity, initialPosition)
    % createPlatform - ...
    %
    % Inputs:
    %   - velocity: ...
    %
    % Outputs:
    %   - scene: ...
    %   - plat: ...

    % Create platform.
    plat = uavPlatform("UAV",scene,"InitialVelocity",initialVelocity,"InitialPosition",initialPosition,"ReferenceFrame","ENU"); 
    updateMesh(plat,"quadrotor",{1.2},[1 0 0],eul2tform([0 0 pi]));
end
