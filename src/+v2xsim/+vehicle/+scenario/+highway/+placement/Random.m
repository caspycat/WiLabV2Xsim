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
                obj, geometry, laneNetwork, randomStream)
            arguments (Input)
                obj (1, 1)
                geometry (1, 1) ...
                    v2xsim.vehicle.scenario.highway.Geometry
                laneNetwork (1, 1) ...
                    v2xsim.vehicle.scenario.highway.LaneNetwork
                randomStream (1, 1) RandStream
            end

            x = geometry.RoadLength .* rand( ...
                randomStream, obj.VehicleCount, 1);
            laneIndex = randi( ...
                randomStream, geometry.NLanes, obj.VehicleCount, 1);
            travelDirection = ...
                2 .* (rand(randomStream, obj.VehicleCount, 1) >= 0.5) - 1;
            laneId = geometry.getMainlineLaneId( ...
                travelDirection, laneIndex);
            progress = x ./ geometry.RoadLength;
            negativeDirection = travelDirection < 0;
            progress(negativeDirection) = ...
                1 - progress(negativeDirection);
            lateralOffsetMeters = zeros(obj.VehicleCount, 1);
            positions = laneNetwork.evaluate(laneId, progress);
            x = positions(:, 1);
            y = positions(:, 2);
            rowNames = compose("Vehicle%d", (1:obj.VehicleCount).');

            vehiclePositions = table( ...
                x, y, laneId, progress, lateralOffsetMeters, ...
                VariableNames=[ ...
                    "X", "Y", "LaneId", "Progress", ...
                    "LateralOffsetMeters"], ...
                RowNames=rowNames);
        end
    end
end
