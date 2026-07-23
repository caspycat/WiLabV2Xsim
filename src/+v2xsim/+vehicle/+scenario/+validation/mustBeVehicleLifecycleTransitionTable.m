function mustBeVehicleLifecycleTransitionTable(value)
%MUSTBEVEHICLELIFECYCLETRANSITIONTABLE Validate lifecycle transitions.
%   The value must be a table containing exactly one single-column variable
%   named Transition. Its values must be VehicleLifecycleTransition enum
%   members.

errorId = "v2xsim:validation:MustBeVehicleLifecycleTransitionTable";

if ~istable(value)
    error(errorId, "Value must be a table.");
end

actualVariables = string(value.Properties.VariableNames);
if ~isequal(actualVariables, "Transition")
    error(errorId, ...
        "Table must contain exactly one variable named Transition.");
end

transitions = value.Transition;
if size(transitions, 2) ~= 1
    error(errorId, ...
        "Table variable Transition must have exactly one column.");
end

expectedClass = "v2xsim.vehicle.scenario.VehicleLifecycleTransition";
if ~isa(transitions, expectedClass)
    error(errorId, ...
        "Table variable Transition must contain %s values.", ...
        expectedClass);
end
end
