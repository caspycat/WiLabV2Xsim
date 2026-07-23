classdef (Abstract) HighwayScenario < ...
        v2xsim.vehicle.scenario.VelocityBasedKinematicScenario
    %HIGHWAYSCENARIO Composes highway placement and mobility strategies.

    properties (Constant)
        CAN_STEP_BACKWARD = false
        PROVIDES_ACCELERATION = false
    end

    properties (SetAccess = immutable)
        Geometry (1, 1) ...
            v2xsim.vehicle.scenario.highway.Geometry = ...
            v2xsim.vehicle.scenario.highway.Geometry(1, 1, 1, 0)
        PlacementStrategy
    end

    properties (SetAccess = private)
        MobilityModel
    end

    properties (Dependent, SetAccess = private)
        NLanes
        LaneWidth
        RoadLength
        CentralDividerWidth
        MeanVehicleSpeed
        VehicleSpeedStandardDeviation
        RerollSpeedOnWrapAround
    end

    methods (Access = protected)
        function obj = HighwayScenario( ...
                geometry, placementStrategy, mobilityModel, randomStream)
            arguments (Input)
                geometry (1, 1) ...
                    v2xsim.vehicle.scenario.highway.Geometry
                placementStrategy (1, 1) ...
                    v2xsim.vehicle.scenario.highway.PlacementStrategy
                mobilityModel (1, 1) ...
                    v2xsim.vehicle.scenario.highway.MobilityModel
                randomStream (1, 1) RandStream
            end

            [placementStrategy, vehiclePlacement] = ...
                placementStrategy.placeVehicles(geometry, randomStream);
            v2xsim.vehicle.scenario.highway.HighwayScenario.validateVehiclePositionTable( ...
                vehiclePlacement);
            vehiclePositions = vehiclePlacement(:, ["X", "Y"]);
            geometry.validateVehiclePositions(vehiclePositions);

            [mobilityModel, initialVehicleKinematics] = ...
                mobilityModel.initializeKinematics( ...
                    geometry, vehiclePlacement);
            v2xsim.vehicle.scenario.highway.HighwayScenario.validateHighwayKinematics( ...
                geometry, mobilityModel, initialVehicleKinematics);

            obj = obj@v2xsim.vehicle.scenario.VelocityBasedKinematicScenario( ...
                initialVehicleKinematics);
            obj.Geometry = geometry;
            obj.PlacementStrategy = placementStrategy;
            obj.MobilityModel = mobilityModel;
        end

        function [obj, vehicleKinematics] = updateVelocities( ...
                obj, vehicleKinematics, deltaTime)
            [mobilityModel, vehicleKinematics] = ...
                obj.MobilityModel.updateVelocities( ...
                    obj.Geometry, vehicleKinematics, deltaTime);
            obj.MobilityModel = mobilityModel;
        end

        function [obj, vehicleKinematics] = resolvePositionConstraints( ...
                obj, previousVehicleKinematics, vehicleKinematics, ...
                deltaTime)
            [mobilityModel, vehicleKinematics] = ...
                obj.MobilityModel.resolvePositionConstraints( ...
                    obj.Geometry, ...
                    previousVehicleKinematics, ...
                    vehicleKinematics, ...
                    deltaTime);
            obj.MobilityModel = mobilityModel;

            v2xsim.vehicle.scenario.highway.HighwayScenario.validateHighwayKinematics( ...
                obj.Geometry, obj.MobilityModel, vehicleKinematics);
        end
    end

    methods
        function value = get.NLanes(obj)
            value = obj.Geometry.NLanes;
        end

        function value = get.LaneWidth(obj)
            value = obj.Geometry.LaneWidth;
        end

        function value = get.RoadLength(obj)
            value = obj.Geometry.RoadLength;
        end

        function value = get.CentralDividerWidth(obj)
            value = obj.Geometry.CentralDividerWidth;
        end

        function value = get.MeanVehicleSpeed(obj)
            value = obj.MobilityModel.MeanVehicleSpeed;
        end

        function value = get.VehicleSpeedStandardDeviation(obj)
            value = obj.MobilityModel.VehicleSpeedStandardDeviation;
        end

        function value = get.RerollSpeedOnWrapAround(obj)
            value = obj.MobilityModel.RerollSpeedOnWrapAround;
        end
    end

    methods (Static, Access = private)
        function validateVehiclePositionTable(vehiclePositions)
            if ~istable(vehiclePositions) || ~all(ismember( ...
                    ["X", "Y"], ...
                    string(vehiclePositions.Properties.VariableNames)))
                error( ...
                    "v2xsim:scenario:highway:InvalidVehiclePositions", ...
                    "A placement strategy must return a table containing " + ...
                    "the variables X and Y.");
            end

            mustBeFloat(vehiclePositions.X);
            mustBeFloat(vehiclePositions.Y);
            mustBeReal(vehiclePositions.X);
            mustBeReal(vehiclePositions.Y);
            mustBeFinite(vehiclePositions.X);
            mustBeFinite(vehiclePositions.Y);

            rowNames = vehiclePositions.Properties.RowNames;
            if height(vehiclePositions) == 0 || isempty(rowNames) || ...
                    numel(rowNames) ~= height(vehiclePositions)
                error( ...
                    "v2xsim:scenario:highway:InvalidVehicleIdentities", ...
                    "A placement strategy must provide at least one " + ...
                    "vehicle and one unique row name per vehicle.");
            end

        end

        function validateHighwayKinematics( ...
                geometry, mobilityModel, vehicleKinematics)
            v2xsim.vehicle.scenario.validation.mustBe2DKinematicsTable( ...
                vehicleKinematics);
            vehiclePositions = vehicleKinematics(:, ["X", "Y"]);
            v2xsim.vehicle.scenario.highway.HighwayScenario.validateVehiclePositionTable( ...
                vehiclePositions);
            geometry.validateVehiclePositions(vehiclePositions);
            mobilityModel.validateKinematics(geometry, vehicleKinematics);
        end
    end
end
