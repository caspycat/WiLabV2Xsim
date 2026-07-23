classdef PositionDelayErrorTest < matlab.unittest.TestCase
    %POSITIONDELAYERRORTEST Tests timestamped apparent-position history.

    methods (Test)
        function testReturnsPositionsFromConfiguredDelay(testCase)
            module = v2xsim.positioning.PositionDelayError(1);
            sampleTimes = [0, 0.5, 1, 1.5];
            apparentX = [0, 5, 10, 15];
            expectedX = [0, 0, 0, 5];

            for sampleIndex = 1:numel(sampleTimes)
                positions = testCase.createPositions( ...
                    apparentX(sampleIndex));
                context = testCase.createContext( ...
                    positions, sampleTimes(sampleIndex));

                [module, delayedPositions] = module.apply( ...
                    positions, context);

                testCase.verifyEqual( ...
                    delayedPositions.X, expectedX(sampleIndex));
            end
        end

        function testUsesSampleAndHoldForUnalignedDelay(testCase)
            module = v2xsim.positioning.PositionDelayError(0.75);
            sampleTimes = [0, 0.5, 1];
            apparentX = [0, 5, 10];

            for sampleIndex = 1:numel(sampleTimes)
                positions = testCase.createPositions( ...
                    apparentX(sampleIndex));
                context = testCase.createContext( ...
                    positions, sampleTimes(sampleIndex));
                [module, delayedPositions] = module.apply( ...
                    positions, context);
            end

            testCase.verifyEqual(delayedPositions.X, 0);
        end

        function testZeroDelayIsIdentity(testCase)
            positions = testCase.createPositions(10);
            module = v2xsim.positioning.PositionDelayError(0);

            [~, delayedPositions] = module.apply( ...
                positions, testCase.createContext(positions, 1));

            testCase.verifyEqual(delayedPositions, positions);
        end

        function testDelaysOutputOfPrecedingModule(testCase)
            addHundred = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(positions, ~) ...
                    testCase.addToX(positions, 100));
            delay = v2xsim.positioning.PositionDelayError(0.5);
            chain = v2xsim.positioning.PositionErrorChain( ...
                {addHundred, delay});
            initialPositions = testCase.createPositions(0);
            laterPositions = testCase.createPositions(5);

            chain = chain.apply( ...
                initialPositions, ...
                testCase.createContext(initialPositions, 0));
            [~, delayedPositions] = chain.apply( ...
                laterPositions, ...
                testCase.createContext(laterPositions, 0.5));

            testCase.verifyEqual(delayedPositions.X, 100);
        end

        function testNewVehicleUsesCurrentPositionUntilHistoryExists( ...
                testCase)
            module = v2xsim.positioning.PositionDelayError(1);
            initialPositions = testCase.createPositions(0);
            module = module.apply( ...
                initialPositions, ...
                testCase.createContext(initialPositions, 0));
            currentPositions = table( ...
                [5; 25], [0; 0], ...
                VariableNames=["X", "Y"], ...
                RowNames=["vehicle-1", "vehicle-2"]);

            [~, delayedPositions] = module.apply( ...
                currentPositions, ...
                testCase.createContext(currentPositions, 0.5));

            testCase.verifyEqual(delayedPositions.X, [0; 25]);
        end

        function testRejectsDecreasingSimulationTime(testCase)
            module = v2xsim.positioning.PositionDelayError(1);
            positions = testCase.createPositions(0);
            module = module.apply( ...
                positions, testCase.createContext(positions, 1));

            testCase.verifyError( ...
                @() module.apply( ...
                    positions, testCase.createContext(positions, 0.5)), ...
                "v2xsim:positioning:PositionDelayTimeReversed");
        end
    end

    methods (Access = private)
        function positions = createPositions(~, x)
            positions = table( ...
                x, 0, ...
                VariableNames=["X", "Y"], ...
                RowNames="vehicle-1");
        end

        function context = createContext( ...
                ~, actualPositions, simulationTimeSeconds)
            context = v2xsim.positioning.PositionErrorContext( ...
                actualPositions, simulationTimeSeconds, 0.5, 10);
        end

        function positions = addToX(~, positions, offset)
            positions.X = positions.X + offset;
        end
    end
end
