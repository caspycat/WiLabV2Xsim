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
end
