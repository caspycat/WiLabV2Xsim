classdef ScenarioStub < v2xsim.vehicle.scenario.Scenario
    %SCENARIOSTUB Controllable concrete Scenario used by unit tests.

    properties (Constant)
        IS_VEHICLE_COUNT_DYNAMIC = false
        CAN_STEP_BACKWARD = false
        PROVIDES_VELOCITY = false
        PROVIDES_ACCELERATION = false
    end

    properties (SetAccess = private)
        NextVehicleKinematics table
        NextVehicleLifecycleTransitions table
        LastDeltaTime (1, 1) double = NaN
    end

    methods
        function obj = ScenarioStub(initialVehicleKinematics, ...
                nextVehicleKinematics, nextVehicleLifecycleTransitions)
            arguments (Input)
                initialVehicleKinematics table
                nextVehicleKinematics table
                nextVehicleLifecycleTransitions table
            end

            obj = obj@v2xsim.vehicle.scenario.Scenario( ...
                initialVehicleKinematics);
            obj.NextVehicleKinematics = nextVehicleKinematics;
            obj.NextVehicleLifecycleTransitions = ...
                nextVehicleLifecycleTransitions;
        end
    end

    methods (Access = protected)
        function [obj, vehicleKinematics, vehicleLifecycleTransitions] = ...
                doStep(obj, deltaTime)
            obj.LastDeltaTime = deltaTime;
            vehicleKinematics = obj.NextVehicleKinematics;
            vehicleLifecycleTransitions = ...
                obj.NextVehicleLifecycleTransitions;
        end
    end
end
