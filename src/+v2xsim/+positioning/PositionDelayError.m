classdef PositionDelayError < v2xsim.positioning.PositionErrorModule
    %POSITIONDELAYERROR Return an earlier apparent position snapshot.
    %   The module stores its apparent input history. At time t, it returns
    %   the newest stored snapshot whose timestamp is no later than
    %   t - DelaySeconds. Before that history exists, it holds the earliest
    %   available snapshot. Vehicles without history are reported at their
    %   current apparent position.

    properties (SetAccess = immutable)
        DelaySeconds (1, 1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
    end

    properties (Access = private)
        HistoryTimesSeconds (:, 1) double = zeros(0, 1)
        HistorySnapshots (1, :) cell = cell(1, 0)
    end

    methods
        function obj = PositionDelayError(delaySeconds)
            arguments (Input)
                delaySeconds (1, 1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
            end

            obj.DelaySeconds = delaySeconds;
        end
    end

    methods (Access = protected)
        function [obj, outputPositions] = doApply( ...
                obj, inputPositions, context)
            currentTime = context.SimulationTimeSeconds;
            obj = obj.storeSnapshot(currentTime, inputPositions);

            targetTime = currentTime - obj.DelaySeconds;
            snapshotIndex = find( ...
                obj.HistoryTimesSeconds <= targetTime, 1, "last");
            if isempty(snapshotIndex)
                snapshotIndex = 1;
            end

            delayedSnapshot = obj.HistorySnapshots{snapshotIndex};
            outputPositions = obj.copyAvailableVehicleHistory( ...
                inputPositions, delayedSnapshot);

            obj.HistoryTimesSeconds = ...
                obj.HistoryTimesSeconds(snapshotIndex:end);
            obj.HistorySnapshots = ...
                obj.HistorySnapshots(snapshotIndex:end);
        end
    end

    methods (Access = private)
        function obj = storeSnapshot(obj, timeSeconds, positions)
            if ~isempty(obj.HistoryTimesSeconds) && ...
                    timeSeconds < obj.HistoryTimesSeconds(end)
                error( ...
                    "v2xsim:positioning:PositionDelayTimeReversed", ...
                    "PositionDelayError requires nondecreasing " + ...
                    "simulation times.");
            end

            if ~isempty(obj.HistoryTimesSeconds) && ...
                    timeSeconds == obj.HistoryTimesSeconds(end)
                obj.HistorySnapshots{end} = positions;
                return
            end

            obj.HistoryTimesSeconds(end + 1, 1) = timeSeconds;
            obj.HistorySnapshots{end + 1} = positions;
        end

        function outputPositions = copyAvailableVehicleHistory( ...
                ~, inputPositions, delayedSnapshot)
            outputPositions = inputPositions;
            currentVehicleIds = string( ...
                inputPositions.Properties.RowNames);
            delayedVehicleIds = string( ...
                delayedSnapshot.Properties.RowNames);
            hasHistory = ismember(currentVehicleIds, delayedVehicleIds);
            vehicleIdsWithHistory = currentVehicleIds(hasHistory);
            outputPositions( ...
                vehicleIdsWithHistory, :) = ...
                delayedSnapshot(vehicleIdsWithHistory, :);
        end
    end
end
