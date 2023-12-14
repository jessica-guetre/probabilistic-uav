% BASIC FLIGHT IN A MAP

rng(0) % For repeatable results.

% Create a scenario that runs for 10 seconds.
s = uavScenario("StopTime",30,"HistoryBufferSize",200);

% Create a quadrotor as the ego vehicle.
egoVelocity = [1 1 0];
egoMultirotor = uavPlatform("EgoVehicle",s,"InitialVelocity", egoVelocity);
updateMesh(egoMultirotor,"quadrotor",{1},[0 1 0],eul2tform([0 0 pi]));

% Set up the 3D view of the scenario.
[ax, plotFrames] = show3D(s);
% Represent the ego UAV as a green marker.
plot3(0,0,0,"Marker","diamond","MarkerFaceColor","green","Parent",plotFrames.EgoVehicle.BodyFrame);
xlim([-5,35]);
ylim([-5,35]);

% Start simulation.
setup(s);
while advance(s)
    % Move ego UAV along its velocity vector.
    motion = read(egoMultirotor);
    motion(1:2) = motion(1:2) + motion(4:5)/s.UpdateRate;
    move(egoMultirotor, motion);

    show3D(s,"FastUpdate", true,"Parent",ax);
    pause(0.02);
end