classdef (Abstract, HandleCompatible) Lane
    %LANE Parametric centerline of one directed highway lane segment.
    %   A lane has a fixed width in meters and maps normalized progress to
    %   Cartesian centerline coordinates in meters. Progress zero is the
    %   start of the lane and progress one is the end. Increasing progress
    %   follows the lane's travel direction.
    %
    %   Concrete lanes implement DOEVALUATE and may represent straight or
    %   curved centerlines. Lane connectivity and Cartesian-to-progress
    %   projection are intentionally outside this initial contract.

    properties (SetAccess = immutable)
        WidthMeters (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBePositive} = 1
    end

    methods (Access = protected)
        function obj = Lane(widthMeters)
            arguments (Input)
                widthMeters (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive}
            end

            obj.WidthMeters = widthMeters;
        end
    end

    methods (Sealed)
        function cartesianPositions = evaluate(obj, progress)
            %EVALUATE Return centerline [X,Y] coordinates in meters.
            %   PROGRESS is a column vector whose values range from zero to
            %   one. The output has one [X,Y] row per progress value.
            arguments (Input)
                obj (1, 1)
                progress (:, 1) double ...
                    {mustBeReal, mustBeFinite}
            end

            if any(progress < 0 | progress > 1, "all")
                error( ...
                    "v2xsim:scenario:highway:ProgressOutsideLane", ...
                    "Lane progress must be between zero and one, " + ...
                    "inclusive.");
            end

            cartesianPositions = obj.doEvaluate(progress);
            obj.validateEvaluationOutput( ...
                progress, cartesianPositions);
        end
    end

    methods (Access = private)
        function validateEvaluationOutput( ...
                ~, progress, cartesianPositions)
            expectedSize = [numel(progress), 2];
            if ~isa(cartesianPositions, "double") || ...
                    ~isequal(size(cartesianPositions), expectedSize) || ...
                    ~isreal(cartesianPositions) || ...
                    any(~isfinite(cartesianPositions), "all")
                error( ...
                    "v2xsim:scenario:highway:InvalidLaneEvaluation", ...
                    "A lane must return one finite, real, double [X,Y] " + ...
                    "row per progress value.");
            end
        end
    end

    methods (Abstract, Access = protected)
        % Map normalized lane progress to Cartesian centerline positions.
        cartesianPositions = doEvaluate(obj, progress)
    end
end
