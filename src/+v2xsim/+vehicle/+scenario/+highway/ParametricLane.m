classdef ParametricLane < v2xsim.vehicle.scenario.highway.Lane
    %PARAMETRICLANE Lane centerline supplied as a function of progress.
    %   EVALUATOR must accept a column vector of normalized progress values
    %   and return one Cartesian [X,Y] row in meters for each value.

    properties (SetAccess = immutable, GetAccess = private)
        Evaluator (1, 1) function_handle = ...
            @(progress) zeros(numel(progress), 2)
    end

    methods
        function obj = ParametricLane(widthMeters, evaluator)
            arguments (Input)
                widthMeters (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBePositive}
                evaluator (1, 1) function_handle
            end

            obj = obj@v2xsim.vehicle.scenario.highway.Lane(widthMeters);
            obj.Evaluator = evaluator;
        end
    end

    methods (Access = protected)
        function cartesianPositions = doEvaluate(obj, progress)
            cartesianPositions = obj.Evaluator(progress);
        end
    end
end
