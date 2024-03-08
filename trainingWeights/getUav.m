function [plat, lidar] = uavSetup(gridScene, initialPosition, updateRate, maxRange, azimuthResolution, elevationLimits, elevationResolution)
    plat = uavPlatform("UAV",gridScene,"InitialPosition",initialPosition,"ReferenceFrame","ENU"); 
    updateMesh(plat,"quadrotor",{1.2},[1 0 0],eul2tform([0 0 pi]));
    lidarModel = uavLidarPointCloudGenerator("UpdateRate",updateRate,"MaxRange",maxRange,"AzimuthResolution",azimuthResolution,"ElevationLimits",elevationLimits,"ElevationResolution",elevationResolution,"HasOrganizedOutput",true);
    lidar = uavSensor("Lidar",plat,lidarModel,MountingLocation=[0,0,-1]);
end