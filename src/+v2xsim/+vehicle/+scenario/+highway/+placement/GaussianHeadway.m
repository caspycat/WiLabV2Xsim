classdef GaussianHeadway < ...
        v2xsim.vehicle.scenario.highway.PlacementStrategy
    %GAUSSIANHEADWAY Positive stochastic circular highway headways.
    %   Headways are sampled independently from a left-truncated normal
    %   distribution, then normalized per lane to sum to RoadLength.

    properties (SetAccess = immutable)
        VehiclesPerLane (1, 1) double ...
            {mustBeInteger, mustBePositive} = 1
        CoefficientOfVariation (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBePositive} = 1 / 3
    end

    methods
        function obj = GaussianHeadway(vehiclesPerLane, options)
            arguments (Input)
                vehiclesPerLane (1, 1) double ...
                    {mustBeInteger, mustBePositive}
                options.CoefficientOfVariation (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive} = 1 / 3
            end

            obj.VehiclesPerLane = vehiclesPerLane;
            obj.CoefficientOfVariation = ...
                options.CoefficientOfVariation;
        end

        function [obj, vehiclePositions] = placeVehicles( ...
                obj, geometry, randomStream)
            arguments (Input)
                obj (1, 1)
                geometry (1, 1) ...
                    v2xsim.vehicle.scenario.highway.Geometry
                randomStream (1, 1) RandStream
            end

            vehicleCount = 2 .* geometry.NLanes .* obj.VehiclesPerLane;
            x = zeros(vehicleCount, 1);
            y = zeros(vehicleCount, 1);

            nextVehicleIndex = 1;
            for travelDirection = [-1, 1]
                for laneIndex = 1:geometry.NLanes
                    indices = nextVehicleIndex:( ...
                        nextVehicleIndex + obj.VehiclesPerLane - 1);
                    x(indices) = obj.createLanePositions( ...
                        geometry.RoadLength, randomStream);
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

    methods (Access = private)
        function positions = createLanePositions( ...
                obj, roadLength, randomStream)
            meanHeadway = roadLength ./ obj.VehiclesPerLane;
            standardDeviation = ...
                meanHeadway .* obj.CoefficientOfVariation;
            headways = ...
                v2xsim.vehicle.scenario.highway.placement.GaussianHeadway.samplePositiveNormal( ...
                    randomStream, ...
                    obj.VehiclesPerLane, ...
                    meanHeadway, ...
                    standardDeviation);

            headways = headways .* (roadLength ./ sum(headways));
            positions = [0; cumsum(headways(1:(end - 1)))];
            phase = roadLength .* rand(randomStream);
            positions = mod(positions + phase, roadLength);
        end
    end

    methods (Static, Access = private)
        function values = samplePositiveNormal( ...
                randomStream, count, meanValue, standardDeviation)
            values = meanValue + ...
                standardDeviation .* randn(randomStream, count, 1);
            valuesToResample = values <= 0;
            while any(valuesToResample)
                values(valuesToResample) = meanValue + ...
                    standardDeviation .* randn( ...
                        randomStream, nnz(valuesToResample), 1);
                valuesToResample = values <= 0;
            end
        end
    end
end
