rng(0) % For repeatable results.

% Create scenario.
simTime = 30;    % in seconds
updateRate = 10;  % in Hz
scene = uavScenario("UpdateRate",updateRate,"StopTime",simTime,"HistoryBufferSize",200);

% Create platform.
velocity = [1 1 0];
plat = uavPlatform("EgoVehicle",scene,"InitialVelocity", velocity,"ReferenceFrame","ENU"); 
updateMesh(plat,"quadrotor",{1},[0 1 0],eul2tform([0 0 pi]));

% Set up the 3D view of the scenario.
[ax, plotFrames] = show3D(scene);
plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.EgoVehicle.BodyFrame);
xlim([-5,35]);
ylim([-5,35]);

% Start simulation.
setup(scene);
while advance(scene)
    % Move ego UAV along its velocity vector.
    motion = read(plat);
    motion(1:2) = motion(1:2) + motion(4:5)/scene.UpdateRate;
    
    % Check if UAV is near the edge of the map and change direction if needed.
    if any(motion(1:2) > 30) || any(motion(1:2) < -5)
        motion(4:5) = rand(1, 2) * 2 - 1; % Random new direction
    end

    move(plat, motion);
    show3D(scene, "FastUpdate", true, "Parent", ax);
    pause(0.02);
end
