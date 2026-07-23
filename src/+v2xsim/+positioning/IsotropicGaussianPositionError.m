classdef IsotropicGaussianPositionError < ...
        v2xsim.positioning.PositionErrorModule
    %ISOTROPICGAUSSIANPOSITIONERROR Random radial position displacement.
    %   Each application draws one zero-mean Gaussian radial displacement
    %   and one uniformly distributed direction per vehicle. The radial
    %   standard deviation is specified in meters.

    properties (SetAccess = immutable)
        StandardDeviationMeters (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        RandomStream (1, 1) RandStream = RandStream.getGlobalStream()
    end

    methods
        function obj = IsotropicGaussianPositionError( ...
                standardDeviationMeters, options)
            arguments (Input)
                standardDeviationMeters (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
                options.RandomStream (1, 1) RandStream = ...
                    RandStream.getGlobalStream()
            end

            obj.StandardDeviationMeters = standardDeviationMeters;
            obj.RandomStream = options.RandomStream;
        end
    end

    methods (Access = protected)
        function [obj, outputPositions] = doApply( ...
                obj, inputPositions, ~)
            outputPositions = inputPositions;
            if obj.StandardDeviationMeters == 0 || ...
                    height(inputPositions) == 0
                return
            end

            radialDisplacement = obj.StandardDeviationMeters .* ...
                randn(obj.RandomStream, height(inputPositions), 1);
            directionRadians = 2 .* pi .* ...
                rand(obj.RandomStream, height(inputPositions), 1);
            outputPositions.X = inputPositions.X + ...
                radialDisplacement .* cos(directionRadians);
            outputPositions.Y = inputPositions.Y + ...
                radialDisplacement .* sin(directionRadians);
        end
    end
end
