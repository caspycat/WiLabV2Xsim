classdef PositionErrorModuleStub < ...
        v2xsim.positioning.PositionErrorModule
    %POSITIONERRORMODULESTUB Configurable position-error test double.

    properties (SetAccess = immutable)
        Transform (1, 1) function_handle = @(value, ~) value
    end

    properties (SetAccess = private)
        ApplicationCount (1, 1) double = 0
        LastSimulationTimeSeconds (1, 1) double = NaN
    end

    methods
        function obj = PositionErrorModuleStub(transform)
            arguments (Input)
                transform (1, 1) function_handle
            end

            obj.Transform = transform;
        end
    end

    methods (Access = protected)
        function [obj, outputPositions] = doApply( ...
                obj, inputPositions, context)
            obj.ApplicationCount = obj.ApplicationCount + 1;
            obj.LastSimulationTimeSeconds = ...
                context.SimulationTimeSeconds;
            outputPositions = ...
                obj.Transform(inputPositions, context);
        end
    end
end
