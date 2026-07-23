function mustBe2DKinematicsTable(value)
%MUSTBE2DKINEMATICSTABLE Validate a table of planar kinematics.
%   The table must contain the single-column floating-point variables X, Y,
%   vX, vY, aX, and aY. All values must be real. X and Y cannot contain
%   NaN; NaN is permitted in velocity and acceleration variables.

if ~istable(value)
    error("v2xsim:validation:MustBe2DKinematicsTable", ...
        "Value must be a table.");
end

requiredVariables = ["X", "Y", "vX", "vY", "aX", "aY"];
actualVariables = string(value.Properties.VariableNames);
missingVariables = setdiff(requiredVariables, actualVariables, "stable");

if ~isempty(missingVariables)
    error("v2xsim:validation:MustBe2DKinematicsTable", ...
        "Table is missing required variable(s): %s.", ...
        join(missingVariables, ", "));
end

for variableName = requiredVariables
    variable = value.(variableName);

    if ~(isa(variable, "single") || isa(variable, "double"))
        error("v2xsim:validation:MustBe2DKinematicsTable", ...
            "Table variable %s must contain floating-point values.", ...
            variableName);
    end

    if size(variable, 2) ~= 1
        error("v2xsim:validation:MustBe2DKinematicsTable", ...
            "Table variable %s must have exactly one column.", variableName);
    end

    if ~isreal(variable)
        error("v2xsim:validation:MustBe2DKinematicsTable", ...
            "Table variable %s must contain real values.", variableName);
    end
end

positionVariables = ["X", "Y"];
for variableName = positionVariables
    if any(isnan(value.(variableName)), "all")
        error("v2xsim:validation:MustBe2DKinematicsTable", ...
            "Table variable %s must not contain NaN.", variableName);
    end
end
end
