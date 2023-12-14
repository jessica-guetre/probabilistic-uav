function [scene, plat] = createPlatform(scene, velocity,initialPosition)
    % createPlatform - ...
    %
    %   [scene, plat] = createPlatform(scene, velocity)
    %
    % Inputs:
    %   - velocity: ...
    %
    % Outputs:
    %   - scene: ...
    %   - plat: ...

    % Create platform.
    plat = uavPlatform("UAV",scene,"InitialVelocity",velocity,"InitialPosition",initialPosition,"ReferenceFrame","ENU"); 
    updateMesh(plat,"quadrotor",{1.2},[1 0 0],eul2tform([0 0 pi]));
end
