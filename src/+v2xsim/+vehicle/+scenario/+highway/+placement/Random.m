classdef Random < v2xsim.vehicle.scenario.highway.PlacementStrategy
    %RANDOM Uniform random longitudinal and lane placement.

    properties (SetAccess = immutable)
        VehicleCount (1, 1) double ...
            {mustBeInteger, mustBePositive} = 1
    end

    methods
        function obj = Random(vehicleCount)
            arguments (Input)
                vehicleCount (1, 1) double ...
                    {mustBeInteger, mustBePositive}
            end

            obj.VehicleCount = vehicleCount;
        end

        function [obj, vehiclePositions] = placeVehicles( ...
                obj, geometry, randomStream)
            arguments (Input)
                obj (1, 1)
                geometry (1, 1) ...
                    v2xsim.vehicle.scenario.highway.Geometry
                randomStream (1, 1) RandStream
            end

            x = geometry.RoadLength .* rand( ...
                randomStream, obj.VehicleCount, 1);
            laneIndex = randi( ...
                randomStream, geometry.NLanes, obj.VehicleCount, 1);
            travelDirection = ...
                2 .* (rand(randomStream, obj.VehicleCount, 1) >= 0.5) - 1;
            distanceFromCenter = geometry.CentralDividerWidth ./ 2 + ...
                (laneIndex - 0.5) .* geometry.LaneWidth;
            y = travelDirection .* distanceFromCenter;
            rowNames = compose("Vehicle%d", (1:obj.VehicleCount).');

            vehiclePositions = table( ...
                x, y, ...
                VariableNames=["X", "Y"], ...
                RowNames=rowNames);
        end
    end
end
