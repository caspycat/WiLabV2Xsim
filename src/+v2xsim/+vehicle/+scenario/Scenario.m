classdef (Abstract, HandleCompatible) Scenario
    %SCENARIO ABS for Traffic Scenarios
    % These are value classes by default – only upgrade to a handle class
    % if dependent on external sources such as TCP Sockets

    properties (Abstract, Constant)
        IS_VEHICLE_COUNT_DYNAMIC (1, 1) logical
        CAN_STEP_BACKWARD (1, 1) logical
        PROVIDES_VELOCITY (1, 1) logical
        PROVIDES_ACCELERATION (1, 1) logical
    end

    properties (SetAccess = private)
        VehicleKinematics table ...
            {v2xsim.vehicle.scenario.validation.mustBe2DKinematicsTable} = ...
            table( ...
                zeros(0, 1), zeros(0, 1), ...
                zeros(0, 1), zeros(0, 1), ...
                zeros(0, 1), zeros(0, 1), ...
                VariableNames=["X", "Y", "vX", "vY", "aX", "aY"])
    end

    methods (Access = protected)
        function obj = Scenario(initialVehicleKinematics)
            arguments (Input)
                initialVehicleKinematics table ...
                    {v2xsim.vehicle.scenario.validation.mustBe2DKinematicsTable}
            end

            obj.validateVehicleRowNames(initialVehicleKinematics);
            obj.VehicleKinematics = initialVehicleKinematics;
        end

    end
    
    methods (Sealed)
        function [obj, vehicleLifecycleTransitions] = step(obj, dt)
            arguments (Input)
                obj (1, 1)
                dt (1, 1) ...
                    {mustBeFloat, mustBeReal, mustBeFinite, mustBeNonzero}
            end

            obj.validateBeforeStep(dt);
            previousVehicleKinematics = obj.VehicleKinematics;

            [obj, nextVehicleKinematics, vehicleLifecycleTransitions] = ...
                obj.doStep(dt);

            obj.validateAfterStep( ...
                previousVehicleKinematics, ...
                nextVehicleKinematics, ...
                vehicleLifecycleTransitions);

            obj.VehicleKinematics = nextVehicleKinematics;
        end
    end

    methods (Access = private)
        function validateBeforeStep(obj, dt)
            arguments (Input)
                obj (1, 1)
                dt (1, 1) ...
                    {mustBeFloat, mustBeReal, mustBeFinite, mustBeNonzero}
            end
            % Validate input arguments of the abstract method,
            % because MATLAB doesn't let us add validation functions for
            % abstract methods

            % Assert that the user does not attempt to step backwards in
            % time if the scenario class defines it cannot step backwards
            if ~obj.CAN_STEP_BACKWARD && dt < 0
                errorId = "v2xsim:scenario:StepBackInTimeNotAllowed";
                errorMessage = ...
                    "Attempted to step %g seconds for a scenario that " + ...
                    "does not allow stepping backwards in time.";
                error(errorId, errorMessage, dt);
            end
        end

        function validateAfterStep(obj, previousVehicleKinematics, ...
                nextVehicleKinematics, vehicleLifecycleTransitions)
            v2xsim.vehicle.scenario.validation.mustBe2DKinematicsTable( ...
                nextVehicleKinematics);
            v2xsim.vehicle.scenario.validation.mustBeVehicleLifecycleTransitionTable( ...
                vehicleLifecycleTransitions);

            obj.validateVehicleRowNames(nextVehicleKinematics);
            obj.validateTransitionRowNames(vehicleLifecycleTransitions);

            % Assert that if the scenario is defined as not having a
            % dynamic vehicle count, then the set of vehicle row names must
            % remain unchanged and all lifecycle transitions must be
            % VehicleLifecycleTransition.Unchanged.
            import v2xsim.vehicle.scenario.VehicleLifecycleTransition;
            if ~obj.IS_VEHICLE_COUNT_DYNAMIC
                previousRowNames = string( ...
                    previousVehicleKinematics.Properties.RowNames);
                nextRowNames = string( ...
                    nextVehicleKinematics.Properties.RowNames);

                hasSameVehicles = isequal( ...
                    sort(previousRowNames), sort(nextRowNames));
                if ~hasSameVehicles
                    errorId = "v2xsim:scenario:VehicleSetChanged";
                    errorMessage = ...
                        "A scenario with a static vehicle count must " + ...
                        "preserve its set of vehicle row names.";
                    error(errorId, errorMessage);
                end

                transitionRowNames = string( ...
                    vehicleLifecycleTransitions.Properties.RowNames);
                transitionsMatchVehicles = isequal( ...
                    sort(nextRowNames), sort(transitionRowNames));
                if ~transitionsMatchVehicles
                    errorId = ...
                        "v2xsim:scenario:LifecycleTransitionSetMismatch";
                    errorMessage = ...
                        "Lifecycle transition row names must match the " + ...
                        "vehicle kinematics row names.";
                    error(errorId, errorMessage);
                end

                actuallyChanged = ~all( ...
                    vehicleLifecycleTransitions.Transition == ...
                    VehicleLifecycleTransition.Unchanged);
                if actuallyChanged
                    errorId = "v2xsim:scenario:DynamicVehicleCountNotAllowed";
                    errorMessage = ...
                        "A scenario with a static vehicle count cannot " + ...
                        "report entered or exited vehicles.";
                    error(errorId, errorMessage);
                end
            end
        end

        function validateVehicleRowNames(~, vehicleKinematics)
            rowNames = vehicleKinematics.Properties.RowNames;
            if isempty(rowNames) || ...
                    numel(rowNames) ~= height(vehicleKinematics)
                errorId = "v2xsim:scenario:MissingVehicleRowNames";
                errorMessage = ...
                    "VehicleKinematics must have one unique row name " + ...
                    "for every vehicle.";
                error(errorId, errorMessage);
            end
        end

        function validateTransitionRowNames(~, vehicleLifecycleTransitions)
            rowNames = vehicleLifecycleTransitions.Properties.RowNames;
            if isempty(rowNames) || ...
                    numel(rowNames) ~= height(vehicleLifecycleTransitions)
                errorId = "v2xsim:scenario:MissingTransitionRowNames";
                errorMessage = ...
                    "VehicleLifecycleTransitions must have one unique " + ...
                    "row name for every transition.";
                error(errorId, errorMessage);
            end
        end
    end

    methods (Abstract, Access = protected)
        % The implementation of the step function
        % Return the updated object, candidate vehicle kinematics, and a
        % lifecycle transition table. Vehicle identities are represented by
        % table row names; row order has no semantic meaning.
        [obj, vehicleKinematics, vehicleLifecycleTransitions] = ...
            doStep(obj, deltaTime)
    end
end
