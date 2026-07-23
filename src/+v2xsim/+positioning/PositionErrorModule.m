classdef (Abstract, HandleCompatible) PositionErrorModule
    %POSITIONERRORMODULE Transforms vehicle positions seen by a controller.
    %   Modules receive a table containing the planar X and Y positions in
    %   meters. Vehicle identities are stored as table row names. The
    %   current simulation time is supplied separately in seconds so that
    %   implementations may be stateful, for example by retaining position
    %   history.
    %
    %   APPLY returns the updated module as well as the transformed
    %   positions. Implementations must preserve the set of vehicle
    %   identities, although row order has no semantic meaning. Modules are
    %   value classes by default; callers must retain the returned module.

    methods (Sealed)
        function [obj, outputPositions] = apply( ...
                obj, inputPositions, simulationTimeSeconds)
            %APPLY Transform one position snapshot.
            arguments (Input)
                obj (1, 1)
                inputPositions table ...
                    {v2xsim.positioning.validation.mustBe2DPositionTable}
                simulationTimeSeconds (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
            end

            [obj, outputPositions] = ...
                obj.doApply(inputPositions, simulationTimeSeconds);

            v2xsim.positioning.validation.mustBe2DPositionTable( ...
                outputPositions);
            obj.validateVehicleIdentities( ...
                inputPositions, outputPositions);
        end
    end

    methods (Access = private)
        function validateVehicleIdentities( ...
                ~, inputPositions, outputPositions)
            inputVehicleIds = string( ...
                inputPositions.Properties.RowNames);
            outputVehicleIds = string( ...
                outputPositions.Properties.RowNames);

            if ~isequal(sort(inputVehicleIds), sort(outputVehicleIds))
                error( ...
                    "v2xsim:positioning:VehicleSetChanged", ...
                    "A position error module must preserve the set of " + ...
                    "vehicle row names.");
            end
        end
    end

    methods (Abstract, Access = protected)
        % Transform inputPositions at simulationTimeSeconds. Return the
        % updated module to support value-class implementations with state.
        [obj, outputPositions] = doApply( ...
            obj, inputPositions, simulationTimeSeconds)
    end
end
