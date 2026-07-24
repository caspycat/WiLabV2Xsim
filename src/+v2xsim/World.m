classdef World
    %WORLD Physical environment shared by traffic and radio subsystems.
    %   A World composes a vehicle TrafficScenario with fixed roadside-unit
    %   positions and physical obstacle geometry. V# for vehicles and RSU#
    %   for roadside units are recommended naming conventions, not
    %   requirements. Names need only be unique across all UEs. The stable
    %   UE order is all vehicles followed by all roadside units.
    %
    %   World represents physical truth. Apparent positions produced by
    %   positioning-error models and radio state do not belong here.
    %   World is a value class, but copy isolation is conditional on its
    %   composed Scenario and ObstacleGeometry implementations: handle
    %   implementations remain shared between World copies.

    properties (SetAccess = private)
        % MATLAB requires a default value when an abstract class is used
        % as a property validator. Construction and private assignment
        % keep this property a scalar Scenario without that invalid
        % default.
        TrafficScenario
        VehicleIds (:, 1) string = strings(0, 1)
    end

    properties (SetAccess = immutable)
        RsuPositions table = table( ...
            zeros(0, 1), zeros(0, 1), ...
            VariableNames=["X", "Y"])
        RsuIds (:, 1) string = strings(0, 1)
        ObstacleGeometry (1,1) ...
            v2xsim.world.ObstacleGeometry = ...
            v2xsim.world.EmptyObstacleGeometry()
    end

    properties (Dependent, SetAccess = private)
        UeIds (:, 1) string
        VehiclePositions table
        UePositions table
    end

    methods
        function obj = World( ...
                trafficScenario,rsuPositions,obstacleGeometry)
            arguments (Input)
                trafficScenario (1, 1) ...
                    v2xsim.vehicle.scenario.Scenario
                rsuPositions table = table( ...
                    zeros(0, 1), zeros(0, 1), ...
                    VariableNames=["X", "Y"])
                obstacleGeometry (1,1) ...
                    v2xsim.world.ObstacleGeometry = ...
                    v2xsim.world.EmptyObstacleGeometry()
            end

            v2xsim.World.validateRsuPositions(rsuPositions);

            vehicleIds = v2xsim.World.idsFromRowNames( ...
                trafficScenario.VehicleKinematics);
            rsuIds = v2xsim.World.idsFromRowNames(rsuPositions);

            v2xsim.World.validateUeIdsAreUnique(vehicleIds, rsuIds);

            obj.TrafficScenario = trafficScenario;
            obj.VehicleIds = vehicleIds;
            obj.RsuPositions = rsuPositions;
            obj.RsuIds = rsuIds;
            obj.ObstacleGeometry = obstacleGeometry;
        end

        function [obj, vehicleLifecycleTransitions] = step(obj, deltaTime)
            arguments (Input)
                obj (1, 1)
                deltaTime (1, 1) ...
                    {mustBeFloat, mustBeReal, mustBeFinite, mustBeNonzero}
            end

            [nextTrafficScenario, vehicleLifecycleTransitions] = ...
                obj.TrafficScenario.step(deltaTime);
            nextVehicleIds = v2xsim.World.idsFromRowNames( ...
                nextTrafficScenario.VehicleKinematics);

            v2xsim.World.validateUeIdsAreUnique( ...
                nextVehicleIds, obj.RsuIds);

            % Preserve the relative order of surviving vehicles. A dynamic
            % traffic scenario may append new identities, but a mere row
            % reorder must never change the World's UE ordering.
            survivingVehicleIds = obj.VehicleIds( ...
                ismember(obj.VehicleIds, nextVehicleIds));
            enteredVehicleIds = nextVehicleIds( ...
                ~ismember(nextVehicleIds, obj.VehicleIds));

            obj.TrafficScenario = nextTrafficScenario;
            obj.VehicleIds = [survivingVehicleIds; enteredVehicleIds];
        end

        function ueIds = get.UeIds(obj)
            ueIds = [obj.VehicleIds; obj.RsuIds];
        end

        function vehiclePositions = get.VehiclePositions(obj)
            vehicleKinematics = obj.TrafficScenario.VehicleKinematics;
            vehiclePositions = vehicleKinematics( ...
                cellstr(obj.VehicleIds), ["X", "Y"]);
        end

        function uePositions = get.UePositions(obj)
            uePositions = [obj.VehiclePositions; obj.RsuPositions];
        end
    end

    methods (Static, Access = private)
        function ids = idsFromRowNames(namedTable)
            ids = string(namedTable.Properties.RowNames);
            ids = ids(:);
        end

        function validateUeIdsAreUnique(vehicleIds, rsuIds)
            ueIds = [vehicleIds; rsuIds];
            if numel(unique(ueIds)) ~= numel(ueIds)
                error( ...
                    "v2xsim:world:DuplicateUeIds", ...
                    "VehicleIds and RsuIds must form a disjoint union.");
            end
        end

        function validateRsuPositions(rsuPositions)
            expectedVariables = ["X", "Y"];
            actualVariables = string( ...
                rsuPositions.Properties.VariableNames);
            if ~isequal(actualVariables, expectedVariables)
                error( ...
                    "v2xsim:world:InvalidRsuPositions", ...
                    "RsuPositions must contain exactly X and Y, in " + ...
                    "that order.");
            end

            for variableName = expectedVariables
                values = rsuPositions.(variableName);
                hasValidValues = ...
                    (isa(values, "single") || isa(values, "double")) && ...
                    size(values, 2) == 1 && ...
                    isreal(values) && all(isfinite(values), "all");
                if ~hasValidValues
                    error( ...
                        "v2xsim:world:InvalidRsuPositions", ...
                        "RsuPositions.%s must be a finite, real, " + ...
                        "single-column floating-point array.", ...
                        variableName);
                end
            end

            rowNames = rsuPositions.Properties.RowNames;
            if height(rsuPositions) > 0 && ...
                    (isempty(rowNames) || ...
                    numel(rowNames) ~= height(rsuPositions))
                error( ...
                    "v2xsim:world:MissingRsuIds", ...
                    "RsuPositions must provide one unique row name for " + ...
                    "every roadside unit.");
            end
        end
    end
end
