classdef LanedScenarioStub < v2xsim.vehicle.scenario.LanedScenario
    %LANEDSCENARIOSTUB Concrete laned scenario used by contract tests.

    properties (Constant)
        CAN_STEP_BACKWARD = true
        PROVIDES_ACCELERATION = false
    end

    methods
        function obj = LanedScenarioStub( ...
                initialVehicleKinematics, laneNetwork, ...
                initialVehicleLaneStates)
            arguments (Input)
                initialVehicleKinematics table
                laneNetwork (1, 1) ...
                    v2xsim.vehicle.scenario.highway.LaneNetwork
                initialVehicleLaneStates table
            end

            obj = obj@v2xsim.vehicle.scenario.LanedScenario( ...
                initialVehicleKinematics, ...
                laneNetwork, ...
                initialVehicleLaneStates);
        end

        function obj = withVehicleLaneStates( ...
                obj, vehicleLaneStates, vehicleKinematics)
            obj = obj.updateVehicleLaneStates( ...
                vehicleLaneStates, vehicleKinematics);
        end
    end

    methods (Access = protected)
        function [obj, vehicleKinematics, vehicleLaneStates] = ...
                resolveLanedPositionConstraints( ...
                    obj, ~, vehicleKinematics, ...
                    previousVehicleLaneStates, ~)
            vehicleLaneStates = previousVehicleLaneStates;
        end
    end
end
