classdef ConstantSpeed < ...
        v2xsim.vehicle.scenario.highway.MobilityModel
    %CONSTANTSPEED Fixed-speed lane motion with periodic wraparound.

    methods
        function obj = ConstantSpeed(speed)
            arguments (Input)
                speed (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
            end

            obj = obj@v2xsim.vehicle.scenario.highway.MobilityModel( ...
                speed, 0, false, RandStream.getGlobalStream());
        end

        function [obj, vehicleKinematics] = initializeKinematics( ...
                obj, ~, vehiclePositions)
            speed = repmat( ...
                obj.MeanVehicleSpeed, height(vehiclePositions), 1);
            vehicleKinematics = ...
                v2xsim.vehicle.scenario.highway.MobilityModel.createKinematics( ...
                    vehiclePositions, speed);
        end

        function [obj, vehicleKinematics] = resolvePositionConstraints( ...
                obj, geometry, previousVehicleKinematics, ...
                vehicleKinematics, ~)
            vehicleKinematics.X = mod( ...
                vehicleKinematics.X, geometry.RoadLength);
            vehicleKinematics.Y = previousVehicleKinematics.Y;
            vehicleKinematics.vY(:) = 0;
        end
    end
end
