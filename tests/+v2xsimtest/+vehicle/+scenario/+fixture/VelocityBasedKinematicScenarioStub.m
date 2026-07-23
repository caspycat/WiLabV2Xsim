classdef VelocityBasedKinematicScenarioStub < ...
        v2xsim.vehicle.scenario.VelocityBasedKinematicScenario
    %VELOCITYBASEDKINEMATICSCENARIOSTUB Test double for integration hooks.

    properties (Constant)
        CAN_STEP_BACKWARD = true
        PROVIDES_ACCELERATION = false
    end

    properties (SetAccess = immutable)
        VelocityDelta (1, 2) double
        PositionCorrection (1, 2) double
        ReorderDuringVelocityUpdate (1, 1) logical
        ChangePositionDuringVelocityUpdate (1, 1) logical
    end

    methods
        function obj = VelocityBasedKinematicScenarioStub( ...
                initialVehicleKinematics, options)
            arguments (Input)
                initialVehicleKinematics table
                options.VelocityDelta (1, 2) double = [0, 0]
                options.PositionCorrection (1, 2) double = [0, 0]
                options.ReorderDuringVelocityUpdate (1, 1) logical = false
                options.ChangePositionDuringVelocityUpdate (1, 1) logical = false
            end

            obj = obj@v2xsim.vehicle.scenario.VelocityBasedKinematicScenario( ...
                initialVehicleKinematics);
            obj.VelocityDelta = options.VelocityDelta;
            obj.PositionCorrection = options.PositionCorrection;
            obj.ReorderDuringVelocityUpdate = ...
                options.ReorderDuringVelocityUpdate;
            obj.ChangePositionDuringVelocityUpdate = ...
                options.ChangePositionDuringVelocityUpdate;
        end
    end

    methods (Access = protected)
        function [obj, vehicleKinematics] = updateVelocities( ...
                obj, vehicleKinematics, ~)
            vehicleKinematics.vX = ...
                vehicleKinematics.vX + obj.VelocityDelta(1);
            vehicleKinematics.vY = ...
                vehicleKinematics.vY + obj.VelocityDelta(2);

            if obj.ChangePositionDuringVelocityUpdate
                vehicleKinematics.X = vehicleKinematics.X + 1;
            end
            if obj.ReorderDuringVelocityUpdate
                vehicleKinematics = flipud(vehicleKinematics);
            end
        end

        function [obj, vehicleKinematics] = resolvePositionConstraints( ...
                obj, ~, vehicleKinematics, ~)
            vehicleKinematics.X = ...
                vehicleKinematics.X + obj.PositionCorrection(1);
            vehicleKinematics.Y = ...
                vehicleKinematics.Y + obj.PositionCorrection(2);
        end
    end
end
