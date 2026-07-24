classdef UniformHeadway < ...
        v2xsim.vehicle.scenario.highway.PlacementStrategy
    %UNIFORMHEADWAY Equal circular headways in every highway lane.

    properties (SetAccess = immutable)
        VehiclesPerLane (1, 1) double ...
            {mustBeInteger, mustBePositive} = 1
    end

    methods
        function obj = UniformHeadway(vehiclesPerLane)
            arguments (Input)
                vehiclesPerLane (1, 1) double ...
                    {mustBeInteger, mustBePositive}
            end

            obj.VehiclesPerLane = vehiclesPerLane;
        end

        function [obj, vehiclePositions] = placeVehicles( ...
                obj, geometry, laneNetwork, ~)
            vehicleCount = 2 .* geometry.NLanes .* obj.VehiclesPerLane;
            headway = geometry.RoadLength ./ obj.VehiclesPerLane;
            x = zeros(vehicleCount, 1);
            laneId = strings(vehicleCount, 1);
            progress = zeros(vehicleCount, 1);

            nextVehicleIndex = 1;
            for travelDirection = [-1, 1]
                for laneIndex = 1:geometry.NLanes
                    indices = nextVehicleIndex:( ...
                        nextVehicleIndex + obj.VehiclesPerLane - 1);
                    x(indices) = ...
                        (0:(obj.VehiclesPerLane - 1)).' .* headway;
                    laneId(indices) = geometry.getMainlineLaneId( ...
                        travelDirection, ...
                        laneIndex .* ones(obj.VehiclesPerLane, 1));
                    progress(indices) = ...
                        x(indices) ./ geometry.RoadLength;
                    if travelDirection < 0
                        progress(indices) = 1 - progress(indices);
                    end
                    nextVehicleIndex = ...
                        nextVehicleIndex + obj.VehiclesPerLane;
                end
            end

            positions = laneNetwork.evaluate(laneId, progress);
            x = positions(:, 1);
            y = positions(:, 2);
            lateralOffsetMeters = zeros(vehicleCount, 1);
            rowNames = compose("V%d", (1:size(x, 1)).');
            vehiclePositions = table( ...
                x, y, laneId, progress, lateralOffsetMeters, ...
                VariableNames=[ ...
                    "X", "Y", "LaneId", "Progress", ...
                    "LateralOffsetMeters"], ...
                RowNames=rowNames);
        end
    end
end
