classdef ExitRampRandom < ...
        v2xsim.vehicle.scenario.highway.PlacementStrategy
    %EXITRAMPRANDOM Random placement across an exit-ramp road network.
    %   X is sampled uniformly over the normal road extent. Outer-lane
    %   vehicles downstream of the fork are placed on either the ramp or
    %   the mainline merge branch using ExitProbability.

    properties (SetAccess = immutable)
        VehicleCount (1, 1) double ...
            {mustBeInteger, mustBePositive} = 1
        ExitProbability (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
    end

    methods
        function obj = ExitRampRandom(vehicleCount, exitProbability)
            arguments (Input)
                vehicleCount (1, 1) double ...
                    {mustBeInteger, mustBePositive}
                exitProbability (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
            end

            if exitProbability > 1
                error( ...
                    "v2xsim:scenario:highway:InvalidExitProbability", ...
                    "ExitProbability must lie between zero and one, " + ...
                    "inclusive.");
            end

            obj.VehicleCount = vehicleCount;
            obj.ExitProbability = exitProbability;
        end

        function [obj, vehiclePlacement] = placeVehicles( ...
                obj, geometry, randomStream)
            arguments (Input)
                obj (1, 1)
                geometry (1, 1) ...
                    v2xsim.vehicle.scenario.highway.ExitRampGeometry
                randomStream (1, 1) RandStream
            end

            x = geometry.RoadLength .* rand( ...
                randomStream, obj.VehicleCount, 1);
            homeLaneIndex = randi( ...
                randomStream, geometry.NLanes, obj.VehicleCount, 1);
            travelDirection = ...
                2 .* (rand(randomStream, obj.VehicleCount, 1) >= 0.5) - 1;
            laneCenters = geometry.getAbsoluteLaneCenters();
            y = travelDirection .* laneCenters(homeLaneIndex).';
            route = repmat("Regular", obj.VehicleCount, 1);

            outerVehicle = homeLaneIndex == geometry.NLanes;
            route(outerVehicle) = "Outer";
            horizontalProgress = travelDirection .* (x - geometry.ForkX);
            downstreamOuterVehicle = outerVehicle & horizontalProgress >= 0;
            exits = downstreamOuterVehicle & ...
                obj.sampleExitDecisions(randomStream, obj.VehicleCount);
            route(exits) = "Ramp";
            y(exits) = travelDirection(exits) .* ...
                (geometry.OuterLaneCenter + horizontalProgress(exits));

            remainsOnMainline = downstreamOuterVehicle & ~exits;
            merges = remainsOnMainline & geometry.MergeDistance > 0 & ...
                horizontalProgress < geometry.MergeDistance;
            route(merges) = "Merge";
            y(merges) = geometry.getMergeY( ...
                travelDirection(merges), horizontalProgress(merges));

            adjacent = remainsOnMainline & ~merges;
            route(adjacent) = "Adjacent";
            y(adjacent) = travelDirection(adjacent) .* ...
                geometry.AdjacentLaneCenter;

            rowNames = compose("Vehicle%d", (1:obj.VehicleCount).');
            vehiclePlacement = table( ...
                x, y, homeLaneIndex, travelDirection, route, ...
                VariableNames=[ ...
                    "X", "Y", "HomeLaneIndex", ...
                    "TravelDirection", "Route"], ...
                RowNames=rowNames);
        end
    end

    methods (Access = private)
        function exits = sampleExitDecisions( ...
                obj, randomStream, vehicleCount)
            if obj.ExitProbability == 0
                exits = false(vehicleCount, 1);
            elseif obj.ExitProbability == 1
                exits = true(vehicleCount, 1);
            else
                exits = rand(randomStream, vehicleCount, 1) < ...
                    obj.ExitProbability;
            end
        end
    end
end
