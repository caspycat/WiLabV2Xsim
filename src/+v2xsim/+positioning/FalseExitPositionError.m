classdef FalseExitPositionError < ...
        v2xsim.positioning.FalseRoutePositionErrorModule
    %FALSEEXITPOSITIONERROR Show merging vehicles on the exit ramp.
    %   Cursed outer-home-lane vehicles that truly merge into the adjacent
    %   mainline lane appear to follow the exit-ramp centerline instead.

    methods
        function obj = FalseExitPositionError(curseProbability, options)
            arguments (Input)
                curseProbability (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
                options.ResetAfterSeconds (1, 1) double ...
                    {mustBeReal, mustBeNonnegative} = Inf
                options.RandomStream (1, 1) RandStream = ...
                    RandStream.getGlobalStream()
            end

            obj = obj@v2xsim.positioning.FalseRoutePositionErrorModule( ...
                curseProbability, ...
                options.ResetAfterSeconds, ...
                options.RandomStream);
        end
    end

    methods (Access = protected)
        function affectedRoute = isAffectedRoute(~, route)
            affectedRoute = route == "Merge" | route == "Adjacent";
        end

        function falsePositions = calculateFalsePositions( ...
                ~, actualPositions, routeStates, context)
            geometry = context.Geometry;
            direction = routeStates.TravelDirection;
            distancePastFork = direction .* ...
                (actualPositions.X - geometry.ForkX);
            rampProgress = distancePastFork ./ ...
                (geometry.RoadLength ./ 2);
            rampProgress = min(max(rampProgress, 0), 1);
            rampLaneIds = geometry.getRampLaneId(direction);
            falsePositions = context.LaneNetwork.evaluate( ...
                rampLaneIds, rampProgress);
        end
    end
end
