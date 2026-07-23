classdef (Abstract, HandleCompatible) MobilityModel
    %MOBILITYMODEL Initializes and advances highway vehicle motion.

    properties (SetAccess = immutable)
        MeanVehicleSpeed (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        VehicleSpeedStandardDeviation (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        RerollSpeedOnWrapAround (1, 1) logical = false
    end

    properties (SetAccess = immutable, GetAccess = protected)
        RandomStream (1, 1) RandStream = RandStream.getGlobalStream()
    end

    methods (Access = protected)
        function obj = MobilityModel( ...
                meanVehicleSpeed, vehicleSpeedStandardDeviation, ...
                rerollSpeedOnWrapAround, randomStream)
            arguments (Input)
                meanVehicleSpeed (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
                vehicleSpeedStandardDeviation (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
                rerollSpeedOnWrapAround (1, 1) logical
                randomStream (1, 1) RandStream
            end

            obj.MeanVehicleSpeed = meanVehicleSpeed;
            obj.VehicleSpeedStandardDeviation = ...
                vehicleSpeedStandardDeviation;
            obj.RerollSpeedOnWrapAround = rerollSpeedOnWrapAround;
            obj.RandomStream = randomStream;
        end
    end

    methods
        function [obj, vehicleKinematics] = updateVelocities( ...
                obj, ~, vehicleKinematics, ~)
            %UPDATEVELOCITIES Default to retaining current velocities.
        end

        function validateKinematics(~, ~, vehicleKinematics)
            if any(vehicleKinematics.vY ~= 0)
                error( ...
                    "v2xsim:scenario:highway:NonzeroLateralVelocity", ...
                    "Straight-highway mobility must keep vY equal to " + ...
                    "zero.");
            end

            movingVehicles = vehicleKinematics.vX ~= 0;
            wrongDirection = ...
                sign(vehicleKinematics.vX(movingVehicles)) ~= ...
                sign(vehicleKinematics.Y(movingVehicles));
            if any(wrongDirection)
                error( ...
                    "v2xsim:scenario:highway:WrongTravelDirection", ...
                    "Positive-Y lanes must travel in +X and negative-Y " + ...
                    "lanes must travel in -X.");
            end
        end
    end

    methods (Abstract)
        [obj, vehicleKinematics] = initializeKinematics( ...
            obj, geometry, vehiclePositions)

        [obj, vehicleKinematics] = resolvePositionConstraints( ...
            obj, geometry, previousVehicleKinematics, ...
            vehicleKinematics, deltaTime)
    end

    methods (Static, Access = protected)
        function vehicleKinematics = createKinematics( ...
                vehiclePositions, speed)
            travelDirection = sign(vehiclePositions.Y);
            vehicleCount = height(vehiclePositions);

            vehicleKinematics = vehiclePositions;
            vehicleKinematics.vX = travelDirection .* speed;
            vehicleKinematics.vY = zeros(vehicleCount, 1);
            vehicleKinematics.aX = NaN(vehicleCount, 1);
            vehicleKinematics.aY = NaN(vehicleCount, 1);
            vehicleKinematics = vehicleKinematics( ...
                :, ["X", "Y", "vX", "vY", "aX", "aY"]);
        end

        function values = sampleNonnegativeNormal( ...
                randomStream, count, meanValue, standardDeviation)
            if standardDeviation == 0
                values = repmat(meanValue, count, 1);
                return
            end

            values = meanValue + ...
                standardDeviation .* randn(randomStream, count, 1);
            valuesToResample = values < 0;
            while any(valuesToResample)
                values(valuesToResample) = meanValue + ...
                    standardDeviation .* randn( ...
                        randomStream, nnz(valuesToResample), 1);
                valuesToResample = values < 0;
            end
        end
    end
end
