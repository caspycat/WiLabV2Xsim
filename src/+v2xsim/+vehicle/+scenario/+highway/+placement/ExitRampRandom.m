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
                obj, geometry, laneNetwork, randomStream)
            arguments (Input)
                obj (1, 1)
                geometry (1, 1) ...
                    v2xsim.vehicle.scenario.highway.ExitRampGeometry
                laneNetwork (1, 1) ...
                    v2xsim.vehicle.scenario.highway.LaneNetwork
                randomStream (1, 1) RandStream
            end

            x = geometry.RoadLength .* rand( ...
                randomStream, obj.VehicleCount, 1);
            homeLaneIndex = randi( ...
                randomStream, geometry.NLanes, obj.VehicleCount, 1);
            travelDirection = ...
                2 .* (rand(randomStream, obj.VehicleCount, 1) >= 0.5) - 1;
            route = repmat("Regular", obj.VehicleCount, 1);

            outerVehicle = homeLaneIndex == geometry.NLanes;
            route(outerVehicle) = "Outer";
            horizontalProgress = travelDirection .* (x - geometry.ForkX);
            downstreamOuterVehicle = outerVehicle & horizontalProgress >= 0;
            exits = downstreamOuterVehicle & ...
                obj.sampleExitDecisions(randomStream, obj.VehicleCount);
            route(exits) = "Ramp";

            remainsOnMainline = downstreamOuterVehicle & ~exits;
            merges = remainsOnMainline & geometry.MergeDistance > 0 & ...
                horizontalProgress < geometry.MergeDistance;
            route(merges) = "Merge";

            adjacent = remainsOnMainline & ~merges;
            route(adjacent) = "Adjacent";

            [laneId, progress] = obj.createLaneCoordinates( ...
                geometry, x, homeLaneIndex, travelDirection, route);
            lateralOffsetMeters = zeros(obj.VehicleCount, 1);
            positions = laneNetwork.evaluate(laneId, progress);
            x = positions(:, 1);
            y = positions(:, 2);

            rowNames = compose("V%d", (1:obj.VehicleCount).');
            vehiclePlacement = table( ...
                x, y, laneId, progress, lateralOffsetMeters, ...
                homeLaneIndex, travelDirection, route, ...
                VariableNames=[ ...
                    "X", "Y", "LaneId", "Progress", ...
                    "LateralOffsetMeters", "HomeLaneIndex", ...
                    "TravelDirection", "Route"], ...
                RowNames=rowNames);
        end
    end

    methods (Access = private)
        function [laneId, progress] = createLaneCoordinates( ...
                obj, geometry, x, homeLaneIndex, ...
                travelDirection, route)
            laneId = strings(obj.VehicleCount, 1);
            progress = zeros(obj.VehicleCount, 1);

            for vehicleIndex = 1:obj.VehicleCount
                direction = travelDirection(vehicleIndex);
                switch route(vehicleIndex)
                    case "Regular"
                        laneId(vehicleIndex) = ...
                            geometry.getMainlineLaneId( ...
                                direction, homeLaneIndex(vehicleIndex));
                        progress(vehicleIndex) = ...
                            obj.getMainlineProgress( ...
                                geometry, x(vehicleIndex), direction);
                    case "Outer"
                        laneId(vehicleIndex) = ...
                            geometry.getOuterApproachLaneId(direction);
                        progress(vehicleIndex) = ...
                            direction .* ...
                            (x(vehicleIndex) - ...
                            obj.getUpstreamX(geometry, direction)) ./ ...
                            geometry.ForkX;
                    case "Ramp"
                        laneId(vehicleIndex) = ...
                            geometry.getRampLaneId(direction);
                        progress(vehicleIndex) = direction .* ...
                            (x(vehicleIndex) - geometry.ForkX) ./ ...
                            (geometry.RoadLength ./ 2);
                    case "Merge"
                        laneId(vehicleIndex) = ...
                            geometry.getMergeLaneId(direction);
                        progress(vehicleIndex) = direction .* ...
                            (x(vehicleIndex) - geometry.ForkX) ./ ...
                            geometry.MergeDistance;
                    case "Adjacent"
                        laneId(vehicleIndex) = ...
                            geometry.getMainlineLaneId( ...
                                direction, geometry.NLanes - 1);
                        progress(vehicleIndex) = ...
                            obj.getMainlineProgress( ...
                                geometry, x(vehicleIndex), direction);
                end
            end

            progress = min(max(progress, 0), 1);
        end

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

    methods (Static, Access = private)
        function progress = getMainlineProgress( ...
                geometry, x, travelDirection)
            if travelDirection > 0
                progress = x ./ geometry.RoadLength;
            else
                progress = 1 - x ./ geometry.RoadLength;
            end
        end

        function upstreamX = getUpstreamX(geometry, travelDirection)
            if travelDirection > 0
                upstreamX = 0;
            else
                upstreamX = geometry.RoadLength;
            end
        end
    end
end
