classdef ExitRampPositionErrorModuleStub < ...
        v2xsim.positioning.ExitRampPositionErrorModule
    %EXITRAMPPOSITIONERRORMODULESTUB Identity ramp-only error module.

    methods (Access = protected)
        function [obj, outputPositions] = doApplyToExitRamp( ...
                obj, inputPositions, ~)
            outputPositions = inputPositions;
        end
    end
end
