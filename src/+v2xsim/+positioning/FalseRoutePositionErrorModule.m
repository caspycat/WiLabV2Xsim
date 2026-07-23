classdef (Abstract) FalseRoutePositionErrorModule < ...
        v2xsim.positioning.ExitRampPositionErrorModule
    %FALSEROUTEPOSITIONERRORMODULE Probabilistic persistent route error.
    %   Each outer-home-lane vehicle is evaluated once. Selected vehicles
    %   remain cursed across later laps. If ResetAfterSeconds is finite,
    %   the timer starts when the false route first differs from the true
    %   route, after which the vehicle is permanently corrected.

    properties (SetAccess = immutable)
        CurseProbability (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
        ResetAfterSeconds (1, 1) double ...
            {mustBeReal, mustBeNonnegative} = Inf
        RandomStream (1, 1) RandStream = RandStream.getGlobalStream()
    end

    properties (Access = private)
        VehicleCurseStates table = table( ...
            false(0, 1), false(0, 1), NaN(0, 1), ...
            VariableNames=[ ...
                "Evaluated", "Cursed", "ActivatedAtSeconds"])
        LastSimulationTimeSeconds (1, 1) double = NaN
    end

    methods (Access = protected)
        function obj = FalseRoutePositionErrorModule( ...
                curseProbability, resetAfterSeconds, randomStream)
            arguments (Input)
                curseProbability (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
                resetAfterSeconds (1, 1) double ...
                    {mustBeReal, mustBeNonnegative}
                randomStream (1, 1) RandStream
            end

            if curseProbability > 1
                error( ...
                    "v2xsim:positioning:InvalidCurseProbability", ...
                    "CurseProbability must lie between zero and one, " + ...
                    "inclusive.");
            end
            if isnan(resetAfterSeconds)
                error( ...
                    "v2xsim:positioning:InvalidResetDuration", ...
                    "ResetAfterSeconds must be nonnegative or Inf.");
            end

            obj.CurseProbability = curseProbability;
            obj.ResetAfterSeconds = resetAfterSeconds;
            obj.RandomStream = randomStream;
        end

        function [obj, outputPositions] = doApplyToExitRamp( ...
                obj, inputPositions, context)
            currentTime = context.SimulationTimeSeconds;
            if ~isnan(obj.LastSimulationTimeSeconds) && ...
                    currentTime < obj.LastSimulationTimeSeconds
                error( ...
                    "v2xsim:positioning:FalseRouteTimeReversed", ...
                    "False-route position errors require " + ...
                    "nondecreasing simulation times.");
            end
            obj.LastSimulationTimeSeconds = currentTime;

            rowNames = inputPositions.Properties.RowNames;
            actualPositions = context.ActualPositions(rowNames, :);
            routeStates = context.VehicleRouteStates(rowNames, :);
            affectedRoute = obj.isAffectedRoute(routeStates.Route);
            [obj, cursed] = obj.updateCurseStates( ...
                routeStates, affectedRoute, currentTime);
            activeError = cursed & affectedRoute;

            outputPositions = inputPositions;
            if ~any(activeError)
                return
            end

            falsePositions = obj.calculateFalsePositions( ...
                actualPositions(activeError, :), ...
                routeStates(activeError, :), ...
                context);
            outputPositions.X(activeError) = ...
                inputPositions.X(activeError) + ...
                falsePositions(:, 1) - ...
                actualPositions.X(activeError);
            outputPositions.Y(activeError) = ...
                inputPositions.Y(activeError) + ...
                falsePositions(:, 2) - ...
                actualPositions.Y(activeError);
        end
    end

    methods (Access = private)
        function [obj, cursed] = updateCurseStates( ...
                obj, routeStates, affectedRoute, currentTime)
            vehicleIds = string(routeStates.Properties.RowNames);
            knownVehicleIds = string( ...
                obj.VehicleCurseStates.Properties.RowNames);
            missingVehicles = ~ismember(vehicleIds, knownVehicleIds);
            if any(missingVehicles)
                missingVehicleIds = vehicleIds(missingVehicles);
                newStates = table( ...
                    false(nnz(missingVehicles), 1), ...
                    false(nnz(missingVehicles), 1), ...
                    NaN(nnz(missingVehicles), 1), ...
                    VariableNames=[ ...
                        "Evaluated", "Cursed", ...
                        "ActivatedAtSeconds"], ...
                    RowNames=cellstr(missingVehicleIds));
                obj.VehicleCurseStates = [ ...
                    obj.VehicleCurseStates; newStates];
                knownVehicleIds = string( ...
                    obj.VehicleCurseStates.Properties.RowNames);
            end

            [~, stateIndices] = ismember(vehicleIds, knownVehicleIds);
            isOuterHomeVehicle = routeStates.Route ~= "Regular";
            shouldEvaluate = isOuterHomeVehicle & ...
                ~obj.VehicleCurseStates.Evaluated(stateIndices);
            if any(shouldEvaluate)
                selected = obj.drawCurses(nnz(shouldEvaluate));
                selectedStateIndices = stateIndices(shouldEvaluate);
                obj.VehicleCurseStates.Evaluated( ...
                    selectedStateIndices) = true;
                obj.VehicleCurseStates.Cursed( ...
                    selectedStateIndices) = selected;
            end

            cursed = obj.VehicleCurseStates.Cursed(stateIndices);
            activatedAt = ...
                obj.VehicleCurseStates.ActivatedAtSeconds(stateIndices);
            shouldActivate = cursed & affectedRoute & isnan(activatedAt);
            if any(shouldActivate)
                activatedStateIndices = stateIndices(shouldActivate);
                obj.VehicleCurseStates.ActivatedAtSeconds( ...
                    activatedStateIndices) = currentTime;
                activatedAt(shouldActivate) = currentTime;
            end

            shouldReset = cursed & ~isnan(activatedAt) & ...
                currentTime - activatedAt >= obj.ResetAfterSeconds;
            if any(shouldReset)
                resetStateIndices = stateIndices(shouldReset);
                obj.VehicleCurseStates.Cursed(resetStateIndices) = false;
                cursed(shouldReset) = false;
            end
        end

        function selected = drawCurses(obj, vehicleCount)
            if obj.CurseProbability == 0
                selected = false(vehicleCount, 1);
            elseif obj.CurseProbability == 1
                selected = true(vehicleCount, 1);
            else
                selected = rand( ...
                    obj.RandomStream, vehicleCount, 1) < ...
                    obj.CurseProbability;
            end
        end
    end

    methods (Abstract, Access = protected)
        affectedRoute = isAffectedRoute(obj, route)
        falsePositions = calculateFalsePositions( ...
            obj, actualPositions, routeStates, context)
    end
end
