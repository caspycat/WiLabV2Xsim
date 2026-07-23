classdef ExitRampGeometry < v2xsim.vehicle.scenario.highway.Geometry
    %EXITRAMPGEOMETRY Bidirectional highway with two symmetric lane drops.
    %   The direction-relative outer lane diverges at X=RoadLength/2 and
    %   follows a 45-degree ramp to the normal X boundary. Vehicles that
    %   remain on the highway move to the adjacent lane over MergeDistance.

    properties (Constant)
        RampAngleDegrees = 45
    end

    properties (SetAccess = immutable)
        MergeDistance (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
    end

    properties (Dependent, SetAccess = private)
        ForkX
        RampLength
        OuterLaneCenter
        AdjacentLaneCenter
    end

    methods
        function obj = ExitRampGeometry( ...
                nLanes, laneWidth, roadLength, centralDividerWidth, ...
                mergeDistance)
            arguments (Input)
                nLanes (1, 1) double ...
                    {mustBeInteger, mustBePositive}
                laneWidth (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive}
                roadLength (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive}
                centralDividerWidth (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
                mergeDistance (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
            end

            if nLanes < 2
                error( ...
                    "v2xsim:scenario:highway:ExitRampNeedsTwoLanes", ...
                    "An exit-ramp highway requires at least two lanes " + ...
                    "per direction.");
            end
            if mergeDistance > roadLength ./ 2
                error( ...
                    "v2xsim:scenario:highway:MergeDistanceTooLong", ...
                    "MergeDistance cannot exceed half of RoadLength.");
            end

            obj = obj@v2xsim.vehicle.scenario.highway.Geometry( ...
                nLanes, laneWidth, roadLength, centralDividerWidth);
            obj.MergeDistance = mergeDistance;
        end

        function value = get.ForkX(obj)
            value = obj.RoadLength ./ 2;
        end

        function value = get.RampLength(obj)
            value = obj.RoadLength ./ sqrt(2);
        end

        function value = get.OuterLaneCenter(obj)
            centers = obj.getAbsoluteLaneCenters();
            value = centers(end);
        end

        function value = get.AdjacentLaneCenter(obj)
            centers = obj.getAbsoluteLaneCenters();
            value = centers(end - 1);
        end

        function laneNetwork = createLaneNetwork(obj)
            %CREATELANENETWORK Create mainline, fork, ramp, and merge lanes.
            laneCountPerDirection = ...
                obj.NLanes + 1 + double(obj.MergeDistance > 0);
            laneIds = strings(1, 2 .* laneCountPerDirection);
            lanes = cell(1, 2 .* laneCountPerDirection);
            laneCenters = obj.getAbsoluteLaneCenters();
            nextLaneIndex = 1;

            for travelDirection = [-1, 1]
                for laneIndex = 1:(obj.NLanes - 1)
                    laneIds(nextLaneIndex) = obj.getMainlineLaneId( ...
                        travelDirection, laneIndex);
                    lanes{nextLaneIndex} = obj.createMainlineLane( ...
                        travelDirection, laneCenters(laneIndex));
                    nextLaneIndex = nextLaneIndex + 1;
                end

                laneIds(nextLaneIndex) = ...
                    obj.getOuterApproachLaneId(travelDirection);
                lanes{nextLaneIndex} = ...
                    obj.createOuterApproachLane(travelDirection);
                nextLaneIndex = nextLaneIndex + 1;

                laneIds(nextLaneIndex) = ...
                    obj.getRampLaneId(travelDirection);
                lanes{nextLaneIndex} = ...
                    obj.createRampLane(travelDirection);
                nextLaneIndex = nextLaneIndex + 1;

                if obj.MergeDistance > 0
                    laneIds(nextLaneIndex) = ...
                        obj.getMergeLaneId(travelDirection);
                    lanes{nextLaneIndex} = ...
                        obj.createMergeLane(travelDirection);
                    nextLaneIndex = nextLaneIndex + 1;
                end
            end

            connections = obj.createLaneConnections();
            laneNetwork = ...
                v2xsim.vehicle.scenario.highway.LaneNetwork( ...
                    laneIds, lanes, connections);
        end

        function laneIds = getOuterApproachLaneId( ...
                ~, travelDirection)
            %GETOUTERAPPROACHLANEID Identify lanes approaching the fork.
            laneIds = ...
                v2xsim.vehicle.scenario.highway.ExitRampGeometry.createSpecialLaneId( ...
                    travelDirection, "outer-approach");
        end

        function laneIds = getRampLaneId(~, travelDirection)
            %GETRAMPLANEID Identify exit-ramp branch lanes.
            laneIds = ...
                v2xsim.vehicle.scenario.highway.ExitRampGeometry.createSpecialLaneId( ...
                    travelDirection, "exit-ramp");
        end

        function laneIds = getMergeLaneId(~, travelDirection)
            %GETMERGELANEID Identify mainline merge connectors.
            laneIds = ...
                v2xsim.vehicle.scenario.highway.ExitRampGeometry.createSpecialLaneId( ...
                    travelDirection, "merge");
        end

        function [x, y] = getRampPosition( ...
                obj, travelDirection, distanceAlongRamp)
            horizontalProgress = distanceAlongRamp ./ sqrt(2);
            x = obj.ForkX + travelDirection .* horizontalProgress;
            y = travelDirection .* ...
                (obj.OuterLaneCenter + horizontalProgress);
        end

        function y = getMergeY( ...
                obj, travelDirection, longitudinalProgress)
            if obj.MergeDistance == 0
                y = travelDirection .* obj.AdjacentLaneCenter;
                return
            end

            normalizedProgress = longitudinalProgress ./ obj.MergeDistance;
            normalizedProgress = min(max(normalizedProgress, 0), 1);
            blend = 10 .* normalizedProgress .^ 3 - ...
                15 .* normalizedProgress .^ 4 + ...
                6 .* normalizedProgress .^ 5;
            absoluteY = obj.OuterLaneCenter + ...
                (obj.AdjacentLaneCenter - obj.OuterLaneCenter) .* blend;
            y = travelDirection .* absoluteY;
        end

        function slope = getMergeLateralSlope( ...
                obj, travelDirection, longitudinalProgress)
            if obj.MergeDistance == 0
                slope = zeros(size(longitudinalProgress));
                return
            end

            normalizedProgress = longitudinalProgress ./ obj.MergeDistance;
            normalizedProgress = min(max(normalizedProgress, 0), 1);
            blendDerivative = ...
                30 .* normalizedProgress .^ 2 - ...
                60 .* normalizedProgress .^ 3 + ...
                30 .* normalizedProgress .^ 4;
            slope = travelDirection .* ...
                (obj.AdjacentLaneCenter - obj.OuterLaneCenter) ./ ...
                obj.MergeDistance .* blendDerivative;
        end

        function validateVehiclePositions(obj, vehiclePositions)
            tolerance = 1e-9 .* max( ...
                [1, obj.RoadLength, obj.OuterLaneCenter]);
            outsideRoad = vehiclePositions.X < -tolerance | ...
                vehiclePositions.X > obj.RoadLength + tolerance;
            if any(outsideRoad)
                error( ...
                    "v2xsim:scenario:highway:PositionOutsideRoad", ...
                    "Exit-ramp X positions must lie between zero and " + ...
                    "RoadLength, inclusive.");
            end

            absoluteY = abs(vehiclePositions.Y);
            laneCenters = obj.getAbsoluteLaneCenters();
            onStraightLane = any( ...
                abs(absoluteY - laneCenters) <= tolerance, 2);

            travelDirection = sign(vehiclePositions.Y);
            horizontalProgress = travelDirection .* ...
                (vehiclePositions.X - obj.ForkX);
            onRampInterval = horizontalProgress >= -tolerance & ...
                horizontalProgress <= obj.RoadLength ./ 2 + tolerance;
            onRamp = onRampInterval & ...
                abs(absoluteY - ...
                    (obj.OuterLaneCenter + horizontalProgress)) <= ...
                    tolerance;

            if obj.MergeDistance == 0
                onMerge = false(height(vehiclePositions), 1);
            else
                onMergeInterval = horizontalProgress >= -tolerance & ...
                    horizontalProgress <= obj.MergeDistance + tolerance;
                expectedMergeY = abs(obj.getMergeY( ...
                    travelDirection, horizontalProgress));
                onMerge = onMergeInterval & ...
                    abs(absoluteY - expectedMergeY) <= tolerance;
            end

            if any(~(onStraightLane | onRamp | onMerge))
                error( ...
                    "v2xsim:scenario:highway:PositionOutsideRoute", ...
                    "Every vehicle must lie on a mainline lane center, " + ...
                    "the exit ramp, or the configured merge path.");
            end
        end
    end

    methods (Access = private)
        function connections = createLaneConnections(obj)
            import v2xsim.vehicle.scenario.highway.LaneConnectionKind

            connectionCountPerDirection = ...
                2 + double(obj.MergeDistance > 0);
            connectionCount = 2 .* connectionCountPerDirection;
            fromLaneId = strings(connectionCount, 1);
            toLaneId = strings(connectionCount, 1);
            kind = repmat( ...
                LaneConnectionKind.Continuation, connectionCount, 1);
            fromProgress = ones(connectionCount, 1);
            toProgress = zeros(connectionCount, 1);
            nextConnectionIndex = 1;

            for travelDirection = [-1, 1]
                approachLaneId = ...
                    obj.getOuterApproachLaneId(travelDirection);
                adjacentLaneId = obj.getMainlineLaneId( ...
                    travelDirection, obj.NLanes - 1);

                fromLaneId(nextConnectionIndex) = approachLaneId;
                toLaneId(nextConnectionIndex) = ...
                    obj.getRampLaneId(travelDirection);
                kind(nextConnectionIndex) = LaneConnectionKind.Diverge;
                nextConnectionIndex = nextConnectionIndex + 1;

                fromLaneId(nextConnectionIndex) = approachLaneId;
                kind(nextConnectionIndex) = LaneConnectionKind.Merge;
                if obj.MergeDistance > 0
                    mergeLaneId = ...
                        obj.getMergeLaneId(travelDirection);
                    toLaneId(nextConnectionIndex) = mergeLaneId;
                else
                    toLaneId(nextConnectionIndex) = adjacentLaneId;
                    toProgress(nextConnectionIndex) = 0.5;
                end
                nextConnectionIndex = nextConnectionIndex + 1;

                if obj.MergeDistance > 0
                    fromLaneId(nextConnectionIndex) = mergeLaneId;
                    toLaneId(nextConnectionIndex) = adjacentLaneId;
                    kind(nextConnectionIndex) = ...
                        LaneConnectionKind.Continuation;
                    toProgress(nextConnectionIndex) = ...
                        0.5 + obj.MergeDistance ./ obj.RoadLength;
                    nextConnectionIndex = nextConnectionIndex + 1;
                end
            end

            connections = table( ...
                fromLaneId, toLaneId, kind, ...
                fromProgress, toProgress, ...
                VariableNames=[ ...
                    "FromLaneId", "ToLaneId", "Kind", ...
                    "FromProgress", "ToProgress"]);
        end

        function lane = createOuterApproachLane( ...
                obj, travelDirection)
            forkX = obj.ForkX;
            roadLength = obj.RoadLength;
            laneCenter = travelDirection .* obj.OuterLaneCenter;
            if travelDirection > 0
                evaluator = @(progress) [ ...
                    forkX .* progress, ...
                    laneCenter .* ones(size(progress))];
            else
                evaluator = @(progress) [ ...
                    roadLength - (roadLength - forkX) .* progress, ...
                    laneCenter .* ones(size(progress))];
            end

            lane = v2xsim.vehicle.scenario.highway.ParametricLane( ...
                obj.LaneWidth, evaluator);
        end

        function lane = createRampLane(obj, travelDirection)
            evaluator = @(progress) [ ...
                obj.ForkX + travelDirection .* ...
                    (obj.RoadLength ./ 2) .* progress, ...
                travelDirection .* (obj.OuterLaneCenter + ...
                    (obj.RoadLength ./ 2) .* progress)];
            lane = v2xsim.vehicle.scenario.highway.ParametricLane( ...
                obj.LaneWidth, evaluator);
        end

        function lane = createMergeLane(obj, travelDirection)
            evaluator = @(progress) [ ...
                obj.ForkX + travelDirection .* ...
                    obj.MergeDistance .* progress, ...
                obj.getMergeY( ...
                    travelDirection .* ones(size(progress)), ...
                    obj.MergeDistance .* progress)];
            lane = v2xsim.vehicle.scenario.highway.ParametricLane( ...
                obj.LaneWidth, evaluator);
        end
    end

    methods (Static, Access = private)
        function laneIds = createSpecialLaneId( ...
                travelDirection, suffix)
            arguments (Input)
                travelDirection (:, 1) double ...
                    {mustBeReal, mustBeFinite}
                suffix (1, 1) string
            end

            if any(abs(travelDirection) ~= 1)
                error( ...
                    "v2xsim:scenario:highway:InvalidTravelDirection", ...
                    "TravelDirection must contain only -1 or 1.");
            end

            directionNames = strings(size(travelDirection));
            directionNames(travelDirection < 0) = "negative-x";
            directionNames(travelDirection > 0) = "positive-x";
            laneIds = directionNames + "-" + suffix;
        end
    end
end
