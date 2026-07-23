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
                obj, geometry, ~)
            vehicleCount = 2 .* geometry.NLanes .* obj.VehiclesPerLane;
            headway = geometry.RoadLength ./ obj.VehiclesPerLane;
            x = zeros(vehicleCount, 1);
            y = zeros(vehicleCount, 1);

            nextVehicleIndex = 1;
            for travelDirection = [-1, 1]
                for laneIndex = 1:geometry.NLanes
                    indices = nextVehicleIndex:( ...
                        nextVehicleIndex + obj.VehiclesPerLane - 1);
                    x(indices) = ...
                        (0:(obj.VehiclesPerLane - 1)).' .* headway;
                    distanceFromCenter = ...
                        geometry.CentralDividerWidth ./ 2 + ...
                        (laneIndex - 0.5) .* geometry.LaneWidth;
                    y(indices) = travelDirection .* distanceFromCenter;
                    nextVehicleIndex = ...
                        nextVehicleIndex + obj.VehiclesPerLane;
                end
            end

            rowNames = compose("Vehicle%d", (1:size(x, 1)).');
            vehiclePositions = table( ...
                x, y, ...
                VariableNames=["X", "Y"], ...
                RowNames=rowNames);
        end
    end
end
