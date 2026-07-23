classdef Geometry
    %GEOMETRY Immutable geometry of a straight bidirectional highway.
    %   NLanes is the number of lanes per travel direction. The highway
    %   extends from X=0 to X=RoadLength, with its center divider at Y=0.

    properties (SetAccess = immutable)
        NLanes (1, 1) double ...
            {mustBeInteger, mustBePositive} = 1
        LaneWidth (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBePositive} = 1
        RoadLength (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBePositive} = 1
        CentralDividerWidth (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
    end

    methods
        function obj = Geometry( ...
                nLanes, laneWidth, roadLength, centralDividerWidth)
            arguments (Input)
                nLanes (1, 1) double ...
                    {mustBeInteger, mustBePositive}
                laneWidth (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive}
                roadLength (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive}
                centralDividerWidth (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
            end

            obj.NLanes = nLanes;
            obj.LaneWidth = laneWidth;
            obj.RoadLength = roadLength;
            obj.CentralDividerWidth = centralDividerWidth;
        end

        function centers = getAbsoluteLaneCenters(obj)
            centers = obj.CentralDividerWidth ./ 2 + ...
                ((1:obj.NLanes) - 0.5) .* obj.LaneWidth;
        end

        function laneNetwork = createLaneNetwork(obj)
            %CREATELANENETWORK Create both directed sets of mainline lanes.
            laneCount = 2 .* obj.NLanes;
            laneIds = strings(1, laneCount);
            lanes = cell(1, laneCount);
            laneCenters = obj.getAbsoluteLaneCenters();

            nextLaneIndex = 1;
            for travelDirection = [-1, 1]
                for laneIndex = 1:obj.NLanes
                    laneIds(nextLaneIndex) = obj.getMainlineLaneId( ...
                        travelDirection, laneIndex);
                    lanes{nextLaneIndex} = obj.createMainlineLane( ...
                        travelDirection, laneCenters(laneIndex));
                    nextLaneIndex = nextLaneIndex + 1;
                end
            end

            laneNetwork = ...
                v2xsim.vehicle.scenario.highway.LaneNetwork( ...
                    laneIds, lanes);
        end

        function laneIds = getMainlineLaneId( ...
                obj, travelDirection, laneIndex)
            %GETMAINLINELANEID Return stable directed mainline identifiers.
            arguments (Input)
                obj (1, 1)
                travelDirection (:, 1) double ...
                    {mustBeReal, mustBeFinite}
                laneIndex (:, 1) double ...
                    {mustBeInteger, mustBePositive}
            end

            if isscalar(travelDirection) && ~isscalar(laneIndex)
                travelDirection = repmat( ...
                    travelDirection, size(laneIndex));
            elseif isscalar(laneIndex) && ~isscalar(travelDirection)
                laneIndex = repmat(laneIndex, size(travelDirection));
            elseif numel(travelDirection) ~= numel(laneIndex)
                error( ...
                    "v2xsim:scenario:highway:LaneIdSizeMismatch", ...
                    "TravelDirection and LaneIndex must contain the " + ...
                    "same number of elements.");
            end
            if any(abs(travelDirection) ~= 1) || ...
                    any(laneIndex > obj.NLanes)
                error( ...
                    "v2xsim:scenario:highway:InvalidMainlineLane", ...
                    "TravelDirection must be -1 or 1 and LaneIndex " + ...
                    "must identify a configured lane.");
            end

            directionNames = strings(size(travelDirection));
            directionNames(travelDirection < 0) = "negative-x";
            directionNames(travelDirection > 0) = "positive-x";
            laneIds = directionNames + "-mainline-" + string(laneIndex);
        end

        function validateVehiclePositions(obj, vehiclePositions)
            outsideRoad = vehiclePositions.X < 0 | ...
                vehiclePositions.X > obj.RoadLength;
            if any(outsideRoad)
                error( ...
                    "v2xsim:scenario:highway:PositionOutsideRoad", ...
                    "Highway X positions must lie between zero and " + ...
                    "RoadLength, inclusive.");
            end

            absoluteLaneCenters = obj.getAbsoluteLaneCenters();
            outsideLane = ~ismember( ...
                abs(vehiclePositions.Y), absoluteLaneCenters);
            if any(outsideLane)
                error( ...
                    "v2xsim:scenario:highway:PositionOutsideLane", ...
                    "Every highway Y position must lie at a configured " + ...
                    "lane center.");
            end
        end
    end

    methods (Access = protected)
        function lane = createMainlineLane( ...
                obj, travelDirection, laneCenter)
            roadLength = obj.RoadLength;
            widthMeters = obj.LaneWidth;
            if travelDirection > 0
                evaluator = @(progress) [ ...
                    roadLength .* progress, ...
                    laneCenter .* ones(size(progress))];
            else
                evaluator = @(progress) [ ...
                    roadLength .* (1 - progress), ...
                    -laneCenter .* ones(size(progress))];
            end

            lane = v2xsim.vehicle.scenario.highway.ParametricLane( ...
                widthMeters, evaluator);
        end
    end
end
