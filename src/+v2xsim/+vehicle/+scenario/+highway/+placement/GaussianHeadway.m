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
                obj, geometry, laneNetwork, randomStream)
            arguments (Input)
                obj (1, 1)
                geometry (1, 1) ...
                    v2xsim.vehicle.scenario.highway.Geometry
                laneNetwork (1, 1) ...
                    v2xsim.vehicle.scenario.highway.LaneNetwork
                randomStream (1, 1) RandStream
            end

            vehicleCount = 2 .* geometry.NLanes .* obj.VehiclesPerLane;
            x = zeros(vehicleCount, 1);
            laneId = strings(vehicleCount, 1);
            progress = zeros(vehicleCount, 1);

            nextVehicleIndex = 1;
            for travelDirection = [-1, 1]
                for laneIndex = 1:geometry.NLanes
                    indices = nextVehicleIndex:( ...
                        nextVehicleIndex + obj.VehiclesPerLane - 1);
                    x(indices) = obj.createLanePositions( ...
                        geometry.RoadLength, randomStream);
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
            rowNames = compose("Vehicle%d", (1:size(x, 1)).');
            vehiclePositions = table( ...
                x, y, laneId, progress, lateralOffsetMeters, ...
                VariableNames=[ ...
                    "X", "Y", "LaneId", "Progress", ...
                    "LateralOffsetMeters"], ...
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
