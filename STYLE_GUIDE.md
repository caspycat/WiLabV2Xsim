# MATLAB Simulator Style Guide

This guide applies to all new MATLAB source and test files added to the simulator. Existing
code should be brought into compliance when it is substantially modified.

The terms **MUST**, **SHOULD**, and **MAY** indicate mandatory rules, preferred rules, and
permitted alternatives.

## 1. Core principles

New code MUST:

1. Be placed in a MATLAB namespace.
2. Expose explicit, validated interfaces.
3. Keep data schemas fixed and inspectable.
4. Prefer pure, independently testable functions.
5. Prefer composition and dependency injection over runtime branching.
6. Minimize shared mutable state.
7. Include automated tests for new behavior.

Correctness and clarity take priority over preserving legacy patterns.

## 2. Repository and namespace structure

All new MATLAB source files MUST be contained in a namespace folder whose name begins with
`+`. Loose production `.m` files at the repository root are prohibited.

Use a structure similar to:

```text
src/
└── +v2xsim/
    ├── +mobility/
    ├── +positioning/
    ├── +resource/
    ├── +scenario/
    └── +internal/

tests/
└── +v2xsimtest/
    ├── +mobility/
    ├── +positioning/
    ├── +resource/
    └── +scenario/
```

`v2xsim` and `v2xsimtest` are example root namespace names. Use the repository's established
root namespace once one has been selected.

Namespace rules:

- Namespace names MUST be lowercase.
- Prefer short semantic namespaces over long concatenated names.
- Use nested namespaces to separate domains.
- Add the parent folder, such as `src`, to the MATLAB path. Do not add `+namespace` folders
  directly.
- Reference members by fully qualified name unless a specific import materially improves
  readability.
- Wildcard imports such as `import v2xsim.resource.*` MUST NOT be used.
- The `+internal` namespace is reserved for implementation details that are not part of the
  supported public API.

Scripts are not permitted in `src`. Executable workflows belong in namespaced functions.
Examples, maintenance utilities, and developer tools should live outside `src`.

## 3. Naming conventions

Names MUST describe domain meaning rather than implementation type.

| Element | Convention | Example |
|---|---|---|
| Namespace | lowercase | `v2xsim.positioning` |
| Function | lower camel case, usually verb-led | `calculateReuseDistance` |
| Method | lower camel case, usually verb-led | `allocateResources` |
| Class | upper camel case, noun or noun phrase | `MaximumReuseDistanceAllocator` |
| Abstract strategy class | upper camel case, capability noun | `ResourceAllocator` |
| Variable | lower camel case | `vehiclePositions` |
| Function argument | lower camel case | `updateIntervalSeconds` |
| Public or private property | upper camel case | `UpdateIntervalSeconds` |
| Name-value argument | upper camel case | `RandomStream` |
| Enumeration class | upper camel case | `TrafficDensity` |
| Enumeration member | upper camel case | `Heavy` |
| Event | upper camel case, preferably past tense | `PositionUpdated` |
| Test class | class or function name plus `Test` | `CalculateReuseDistanceTest` |
| Test method | `test` plus expected behavior | `testReturnsZeroForSamePosition` |

Additional naming rules:

- Functions and methods SHOULD begin with a verb: `calculate`, `create`, `load`, `validate`,
  `select`, or `update`.
- Classes SHOULD be nouns. Do not prefix interfaces or abstract classes with `I`.
- Boolean names SHOULD read as predicates: `isValid`, `hasPositionUpdate`, `canTransmit`,
  or `shouldReschedule`.
- Collections SHOULD use plural names: `vehicles`, `resourceIndices`, `positionUpdates`.
- Include units when a quantity would otherwise be ambiguous:
  `delaySeconds`, `distanceMeters`, `speedMetersPerSecond`.
- Avoid one-letter names except for brief mathematical expressions or short loop indices.
- Domain abbreviations MAY be used when they are widely understood in the project. Treat
  acronyms as words when composing names: `gnssError`, `vehicleId`, `PrrResult`,
  `GnssErrorModel`.
- Do not encode MATLAB types in names. Avoid names such as `vehicleStruct`, `positionArray`,
  or `allocatorObj` unless the type distinction is itself meaningful.
- Do not shadow MATLAB functions, keywords, or common built-ins.
- Do not use `i` or `j` as the imaginary unit. Use `1i` or `1j` explicitly.

The filename MUST exactly match its primary function or class name, including case.

## 4. Function design

Functions SHOULD be small, cohesive, and deterministic.

A function SHOULD:

- Perform one clearly described operation.
- Return its result instead of mutating unrelated state.
- Receive dependencies explicitly as arguments.
- Separate computation from file access, plotting, logging, randomness, and global state.
- Be directly testable without running a complete simulation.

Prefer extracting reusable calculations into namespaced functions rather than hiding them in
large scripts, callbacks, or nested functions.

Nested functions SHOULD NOT be introduced. They cannot use argument validation syntax and
are difficult to test independently.

### 4.1 Argument validation

Every production function, constructor, and concrete method that accepts inputs MUST declare
them in one or more `arguments` blocks.

Each caller-provided input MUST specify an appropriate combination of:

- Size
- Class
- Built-in validator
- Project-specific validator
- Default value, when optional

Example:

```matlab
function distanceMeters = calculateReuseDistance(positionsMeters, options)
    arguments
        positionsMeters (:,2) double {mustBeReal, mustBeFinite}
        options.MinimumDistanceMeters (1,1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative} = 0
    end

    distanceMeters = ...
        v2xsim.resource.internal.calculatePairwiseDistance(positionsMeters);

    distanceMeters = max(distanceMeters, options.MinimumDistanceMeters);
end
```

Rules:

- Do not use `inputParser` in new code.
- Do not use manual `nargin` branching to implement optional arguments.
- Name-value arguments MUST be declared through an `arguments` block.
- Calls SHOULD use the full name-value argument name rather than partial matching.
- Custom validators MUST be side-effect free and return clear errors.
- Output `arguments (Output)` blocks SHOULD be used for stable public APIs when output class
  or shape is part of the contract.
- Validation must occur at component boundaries. Internal code may rely on already validated
  invariants, but should still use `arguments` blocks where MATLAB permits them.

MATLAB does not permit `arguments` blocks in abstract methods, nested functions, or handle
class destructors. Therefore:

- Abstract methods MUST document their input and output contract.
- Every concrete implementation MUST validate that contract.
- Nested functions SHOULD be refactored into namespaced functions.
- Destructors are exempt from this rule.
- Framework-controlled callbacks and test entry points MAY omit redundant validation when
  their signature is fixed by MATLAB.

## 5. Classes and data modelling

### 5.1 Prefer value classes

Classes are value classes by default and SHOULD remain value classes.

Use a value class for:

- Configuration
- Simulation parameters
- Vehicle state snapshots
- Position updates
- Resource-allocation results
- Immutable domain values
- Data transferred between components

A method that changes a value object MUST return the updated object:

```matlab
state = state.withPosition(newPositionMeters);
```

A class MAY derive from `handle` only when reference identity is required, such as:

- A unique external resource
- Shared mutable runtime state that cannot reasonably be returned
- Event and listener support
- A lifecycle-managed service
- A graph node or object whose identity is distinct from its values

A new handle class MUST state in its class documentation why value semantics are unsuitable.
Do not choose handle semantics merely to avoid returning an updated object.

### 5.2 Properties

Externally settable properties MUST declare size, class, and validation where applicable.

Prefer restrictive access:

- Use immutable or private-set properties for configuration and identity.
- Expose methods that preserve invariants instead of allowing arbitrary property mutation.
- Avoid public mutable properties for shared simulation state.
- Computed values SHOULD use dependent properties only when they remain inexpensive and
  unsurprising.

Example:

```matlab
classdef PositionUpdate
    properties (SetAccess = immutable)
        VehicleId (1,1) uint64
        PositionMeters (1,2) double {mustBeReal, mustBeFinite}
        TimestampSeconds (1,1) double ...
            {mustBeReal, mustBeFinite, mustBeNonnegative}
    end

    methods
        function obj = PositionUpdate(vehicleId, positionMeters, timestampSeconds)
            arguments
                vehicleId (1,1) uint64
                positionMeters (1,2) double {mustBeReal, mustBeFinite}
                timestampSeconds (1,1) double ...
                    {mustBeReal, mustBeFinite, mustBeNonnegative}
            end

            obj.VehicleId = vehicleId;
            obj.PositionMeters = positionMeters;
            obj.TimestampSeconds = timestampSeconds;
        end
    end
end
```

### 5.3 No god structs or dynamic field schemas

A struct MUST NOT be used as a general-purpose object whose possible fields are undocumented,
mutable, or discovered only at runtime.

Prohibited patterns include:

```matlab
state.(fieldName) = value;
config.(algorithmName) = parameters;
```

Structs MAY be used when all fields are fixed, local, and obvious by inspection, including the
validated options struct produced by an `arguments` block.

For larger or shared schemas, use:

- A validated value class for domain data or configuration
- An enumeration for a finite set of states
- A `table` or `timetable` for tabular observations
- A `dictionary` or `containers.Map` when arbitrary keys are an intentional requirement

The schema of data passed between simulator components MUST be statically visible in class,
table, or function definitions.

## 6. Dependency injection and runtime behavior

Prefer inversion of control and composition over selecting behavior with `if`, `elseif`, or
`switch` statements throughout the simulator.

Dynamic behavior SHOULD be supplied as:

- An object implementing an abstract strategy contract
- A function handle
- A validated configuration object containing injected collaborators

Example:

```matlab
function result = runAllocation(allocator, snapshot)
    arguments
        allocator (1,1) v2xsim.resource.ResourceAllocator
        snapshot (1,1) v2xsim.state.NetworkSnapshot
    end

    result = allocator.allocate(snapshot);
end
```

The simulation core MUST NOT contain branches such as:

```matlab
switch algorithmName
    case "mrd"
        ...
    case "mode2"
        ...
end
```

Resolve configured names to concrete implementations once, at a composition root or factory,
then inject the resulting dependency.

Factories MAY use limited branching when translating an external configuration into a
validated object. Such branching MUST be centralized, exhaustive, and covered by tests.

Avoid:

- Service locator patterns
- Global variables
- Persistent mutable dependencies
- Singleton objects used as general shared state
- Arbitrary `eval`, `evalin`, `assignin`, or string-based execution

## 7. Randomness, time, and external state

Simulation code must remain reproducible.

- Random behavior SHOULD receive a `RandStream` or equivalent dependency rather than use the
  global random stream implicitly.
- Tests MUST use deterministic streams or fixed seeds and restore any modified global state.
- File paths, clocks, loggers, and external services SHOULD be passed into the component that
  uses them.
- Functions MUST NOT change the MATLAB path, working directory, warning state, graphics
  state, or global random state without restoring it.
- Do not place `clear`, `clc`, or `close all` inside reusable functions.

## 8. Unit tests

Every new public function, concrete class, and bug fix MUST have corresponding automated
tests. Nontrivial internal functions SHOULD be tested directly when they represent an
independent unit.

Use class-based tests derived from `matlab.unittest.TestCase`.

Tests MUST:

- Be namespaced.
- Run independently and in any order.
- Be deterministic and repeatable.
- Avoid dependence on developer-specific absolute paths.
- Restore modified state through fixtures, teardown, or `onCleanup`.
- Test both valid behavior and rejected invalid inputs.
- Use tolerances for floating-point comparisons.
- Reproduce every fixed bug with a regression test.

Mirror source domains in the test namespace:

```text
src/+v2xsim/+resource/calculateReuseDistance.m
tests/+v2xsimtest/+resource/CalculateReuseDistanceTest.m
```

A test method SHOULD describe one observable behavior:

```matlab
classdef CalculateReuseDistanceTest < matlab.unittest.TestCase
    methods (Test)
        function testReturnsZeroForIdenticalPositions(testCase)
            positionsMeters = [10, 20; 10, 20];

            actualDistanceMeters = ...
                v2xsim.resource.calculateReuseDistance(positionsMeters);

            testCase.verifyEqual(actualDistanceMeters, 0);
        end
    end
end
```

Prefer testing through public behavior. Extract difficult calculations into pure functions
rather than exposing private state solely for testing.

Use integration tests for interactions between simulator components and system tests for
complete scenarios. A unit test should not need to run an entire simulation.

## 9. Text, formatting, and MATLAB idioms

- Use four spaces for indentation. Do not use tabs.
- Use one statement per line and terminate non-display statements with semicolons.
- Use blank lines to separate logical stages, not every statement.
- Wrap long expressions at meaningful operators or argument boundaries.
- Prefer string scalars and string arrays in new APIs. Use character vectors only when an
  external MATLAB API requires them.
- Use `&&` and `||` for scalar logical conditions; use `&` and `|` for element-wise logical
  operations.
- Parenthesize compound conditions when precedence is not immediately obvious.
- Avoid clever vectorization that obscures intent. Profile before adding complexity for
  performance.
- Preallocate arrays in measured performance-critical loops.
- Do not suppress Code Analyzer warnings without a nearby explanation.
- A suppression MUST be as narrow as possible.

## 10. Documentation and comments

Every public function and class MUST have a help comment.

A function help comment SHOULD state:

- What the function does
- Input and output meaning
- Units and coordinate conventions
- Important assumptions
- Name-value options
- Errors callers are expected to handle

Comments SHOULD explain why a decision exists, not restate the code.

Document coordinate systems explicitly. For example, state whether positive `y` points
upward or downward, whether angles are clockwise or counterclockwise, and whether positions
are local or geographic.

Mathematical notation MAY use short variable names inside a small, clearly delimited formula.
The surrounding interface MUST still use descriptive names.

## 11. Errors and diagnostics

- Prefer argument validators for caller input errors.
- Use `assert` only for internal invariants that indicate a programming defect.
- Error and warning identifiers MUST be namespaced:

```matlab
error("v2xsim:resource:InvalidAllocation", ...
    "Resource allocation contains duplicate vehicle assignments.");
```

- Messages MUST identify the invalid value or violated invariant and suggest corrective
  action when practical.
- Do not catch an exception merely to ignore it.
- Wrap an exception only when adding meaningful domain context, and preserve the cause.

## 12. Code review checklist

Before merging new MATLAB code, verify that:

- [ ] Every production file is namespaced.
- [ ] Filenames match their primary function or class.
- [ ] Names follow the casing and semantic rules.
- [ ] Public inputs use `arguments` blocks with meaningful validation.
- [ ] Data schemas are fixed and inspectable.
- [ ] No god struct or dynamic-field control flow was introduced.
- [ ] Value semantics were used unless handle identity is justified.
- [ ] Runtime behavior is injected rather than repeatedly selected by branching.
- [ ] Randomness and external state are controllable and reproducible.
- [ ] Unit tests cover normal behavior, edge cases, and invalid inputs.
- [ ] Tests are deterministic and independent.
- [ ] Code Analyzer warnings are resolved or narrowly justified.
- [ ] Public behavior, units, and coordinate conventions are documented.

## 13. Optional Code Analyzer naming configuration

Repositories using a MATLAB release that supports `codeAnalyzerConfiguration.json` SHOULD
configure naming checks similar to:

```json
{
  "naming": {
    "variable": {
      "casing": "lowerCamelCase"
    },
    "function": {
      "casing": "lowerCamelCase"
    },
    "localFunction": {
      "casing": "lowerCamelCase"
    },
    "nestedFunction": {
      "casing": "lowerCamelCase"
    },
    "class": {
      "casing": "UpperCamelCase"
    },
    "property": {
      "casing": "UpperCamelCase"
    },
    "method": {
      "casing": [
        "lowerCamelCase",
        "UpperCamelCase"
      ]
    },
    "event": {
      "casing": "UpperCamelCase"
    },
    "enumeration": {
      "casing": "UpperCamelCase"
    }
  }
}
```

`UpperCamelCase` is allowed for methods because MATLAB constructors must match their class
name. Code review must still enforce lower camel case for non-constructor methods.
