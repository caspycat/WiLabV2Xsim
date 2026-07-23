function [indexNewVehicles,indexOldVehicles,indexOldVehiclesToOld, ...
        IDvehicleExit,positionManagement,simValues] = updatePosition( ...
        time,IDvehicle,updateInterval,positionManagement,simValues, ...
        outParams,~)
%UPDATEPOSITION Step the configured Scenario and update legacy matrices.
%   Scenario vehicle identity comes from table row names. The mapping
%   captured by initVehiclePositions keeps those identities associated with
%   stable legacy numeric IDs even if a Scenario changes its table row
%   order. RSUs and other legacy-managed stations are left untouched.

scenario = simValues.scenario;
if scenario.IS_VEHICLE_COUNT_DYNAMIC
    error('v2xsim:legacy:DynamicVehicleCountUnsupported', ...
        ['The legacy simulator requires a fixed vehicle count. ', ...
        'Use a Scenario whose IS_VEHICLE_COUNT_DYNAMIC flag is false.']);
end

[scenario,~] = scenario.step(updateInterval);
simValues.scenario = scenario;

vehicleKinematics = scenario.VehicleKinematics( ...
    simValues.scenarioVehicleRowNames,:);
scenarioVehicleIDs = simValues.scenarioVehicleIDs;

if height(vehicleKinematics)~=numel(scenarioVehicleIDs)
    error('v2xsim:legacy:ScenarioVehicleMappingSizeMismatch', ...
        ['The number of Scenario rows no longer matches the number of ', ...
        'legacy vehicle IDs captured during initialization.']);
end

positionManagement.XvehicleReal(scenarioVehicleIDs) = ...
    vehicleKinematics.X;
positionManagement.YvehicleReal(scenarioVehicleIDs) = ...
    vehicleKinematics.Y;

speed = hypot(vehicleKinematics.vX,vehicleKinematics.vY);
direction = complex(zeros(height(vehicleKinematics),1));
movingVehicles = speed>0;
direction(movingVehicles) = complex( ...
    vehicleKinematics.vX(movingVehicles)./speed(movingVehicles), ...
    vehicleKinematics.vY(movingVehicles)./speed(movingVehicles));

positionManagement.v(scenarioVehicleIDs) = speed;
positionManagement.direction(scenarioVehicleIDs) = direction;

% Compatibility aliases used by integrations outside this checkout.
positionManagement.XVehicle = positionManagement.XvehicleReal;
positionManagement.YVehicle = positionManagement.YvehicleReal;

% All currently supported Scenarios have a fixed vehicle set, so every
% active station remains at the same position in the active-ID vectors.
indexNewVehicles = [];
indexOldVehicles = (1:numel(IDvehicle)).';
indexOldVehiclesToOld = indexOldVehicles;
IDvehicleExit = [];

if ~isempty(outParams) && outParams.printSpeed
    printSpeedToFile( ...
        time,IDvehicle,positionManagement.v,simValues.maxID,outParams);
end
end
