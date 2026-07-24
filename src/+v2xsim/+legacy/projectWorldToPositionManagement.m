function positionManagement = projectWorldToPositionManagement( ...
        world,positionManagement)
%PROJECTWORLDTOPOSITIONMANAGEMENT Project physical truth to legacy arrays.
%   Dense legacy UE indices follow World.UeIds exactly. Vehicle speed and
%   direction are projected from the traffic scenario; fixed RSUs have
%   zero speed and direction.

arguments (Input)
    world (1,1) v2xsim.World
    positionManagement (1,1) struct
end

uePositions = world.UePositions;
ueCount = numel(world.UeIds);

positionManagement.XvehicleReal = uePositions.X;
positionManagement.YvehicleReal = uePositions.Y;
positionManagement.v = zeros(ueCount,1);
positionManagement.direction = complex(zeros(ueCount,1));

[hasVehicleId,vehicleUeIndices] = ...
    ismember(world.VehicleIds,world.UeIds);
if ~all(hasVehicleId)
    error( ...
        'v2xsim:legacy:InvalidWorldIdentity', ...
        'Every World.VehicleId must occur in World.UeIds.');
end

vehicleKinematics = world.TrafficScenario.VehicleKinematics( ...
    cellstr(world.VehicleIds),:);
speed = hypot(vehicleKinematics.vX,vehicleKinematics.vY);
direction = complex(zeros(height(vehicleKinematics),1));
movingVehicles = speed>0;
direction(movingVehicles) = complex( ...
    vehicleKinematics.vX(movingVehicles)./speed(movingVehicles), ...
    vehicleKinematics.vY(movingVehicles)./speed(movingVehicles));

positionManagement.v(vehicleUeIndices) = speed;
positionManagement.direction(vehicleUeIndices) = direction;

% Compatibility aliases used by integrations outside this checkout.
positionManagement.XVehicle = positionManagement.XvehicleReal;
positionManagement.YVehicle = positionManagement.YvehicleReal;
end
