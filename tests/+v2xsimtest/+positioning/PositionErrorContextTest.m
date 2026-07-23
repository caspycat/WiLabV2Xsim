classdef PositionErrorContextTest < matlab.unittest.TestCase
    %POSITIONERRORCONTEXTTEST Tests generic error-context invariants.

    methods (Test)
        function testStoresActualPositionsAndTiming(testCase)
            actualPositions = testCase.createPositions();

            context = v2xsim.positioning.PositionErrorContext( ...
                actualPositions, 2.5, 0.1, 10);

            testCase.verifyEqual( ...
                context.ActualPositions, actualPositions);
            testCase.verifyEqual(context.SimulationTimeSeconds, 2.5);
            testCase.verifyEqual(context.TimeStepSeconds, 0.1);
            testCase.verifyEqual( ...
                context.SimulationDurationSeconds, 10);
        end

        function testRejectsTimeBeyondDuration(testCase)
            constructor = @() ...
                v2xsim.positioning.PositionErrorContext( ...
                    testCase.createPositions(), 10.1, 0.1, 10);

            testCase.verifyError( ...
                constructor, ...
                "v2xsim:positioning:TimeBeyondSimulationDuration");
        end
    end

    methods (Access = private)
        function positions = createPositions(~)
            positions = table( ...
                [10; 20], [30; 40], ...
                VariableNames=["X", "Y"], ...
                RowNames=["vehicle-1", "vehicle-2"]);
        end
    end
end
