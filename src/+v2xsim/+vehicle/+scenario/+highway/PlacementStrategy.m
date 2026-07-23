classdef (Abstract, HandleCompatible) PlacementStrategy
    %PLACEMENTSTRATEGY Creates initial highway vehicle positions.

    methods (Abstract)
        % Return a table containing X, Y, LaneId, Progress, and
        % LateralOffsetMeters, with vehicle identities stored as row names.
        % Additional variables may carry placement metadata needed to
        % initialize a compatible mobility model.
        [obj, vehiclePositions] = placeVehicles( ...
            obj, geometry, laneNetwork, randomStream)
    end
end
