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
end
