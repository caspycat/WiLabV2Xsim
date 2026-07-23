classdef (Abstract, HandleCompatible) PositionErrorModule
    %POSITIONERRORMODULE Transforms vehicle positions seen by a controller.
    %   Modules receive a table containing the planar X and Y positions in
    %   meters. Vehicle identities are stored as table row names. Context
    %   supplies timing and the unmodified scenario position snapshot.
    %   Specialized contexts may provide additional scenario capabilities.
    %
    %   APPLY returns the updated module as well as the transformed
    %   positions. Implementations must preserve the set of vehicle
    %   identities, although row order has no semantic meaning. Modules are
    %   value classes by default; callers must retain the returned module.

    methods (Sealed)
        function [obj, outputPositions] = apply( ...
                obj, inputPositions, context)
            %APPLY Transform one position snapshot.
            arguments (Input)
                obj (1, 1)
                inputPositions table ...
                    {v2xsim.positioning.validation.mustBe2DPositionTable}
                context (1, 1) ...
                    v2xsim.positioning.PositionErrorContext
            end

            obj.validateContextVehicleIdentities( ...
                inputPositions, context.ActualPositions);
            [obj, outputPositions] = ...
                obj.doApply(inputPositions, context);

            v2xsim.positioning.validation.mustBe2DPositionTable( ...
                outputPositions);
            obj.validateVehicleIdentities( ...
                inputPositions, outputPositions);
        end
    end

    methods (Access = private)
        function validateContextVehicleIdentities( ...
                ~, inputPositions, actualPositions)
            inputVehicleIds = string( ...
                inputPositions.Properties.RowNames);
            actualVehicleIds = string( ...
                actualPositions.Properties.RowNames);

            if ~isequal(sort(inputVehicleIds), sort(actualVehicleIds))
                error( ...
                    "v2xsim:positioning:ContextVehicleSetMismatch", ...
                    "Apparent and actual positions must identify the " + ...
                    "same vehicles.");
            end
        end

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
        % Transform the apparent inputPositions using the immutable
        % context. Return the updated module to support value-class
        % implementations with state.
        [obj, outputPositions] = doApply( ...
            obj, inputPositions, context)
    end
end
