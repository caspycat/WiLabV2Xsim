classdef LaneStub < v2xsim.vehicle.scenario.highway.Lane
    %LANESTUB Configurable parametric lane used by unit tests.

    properties (SetAccess = immutable)
        Evaluator (1, 1) function_handle = ...
            @(progress) zeros(numel(progress), 2)
    end

    methods
        function obj = LaneStub(widthMeters, evaluator)
            arguments (Input)
                widthMeters (1, 1) double
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
