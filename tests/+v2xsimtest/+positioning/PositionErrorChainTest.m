classdef PositionErrorChainTest < matlab.unittest.TestCase
    %POSITIONERRORCHAINTEST Tests ordered position-error composition.

    methods (Test)
        function testAppliesModulesFromLeftToRight(testCase)
            addOne = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) testCase.addToX(value, 1));
            doubleX = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) testCase.multiplyX(value, 2));
            chain = v2xsim.positioning.PositionErrorChain( ...
                {addOne, doubleX});
            positions = testCase.createPositions();
            context = testCase.createContext(positions, 3);

            [chain, actualPositions] = chain.apply(positions, context);

            expectedPositions = positions;
            expectedPositions.X = (positions.X + 1) .* 2;
            testCase.verifyEqual(actualPositions, expectedPositions);
            testCase.verifyEqual( ...
                chain.Modules{1}.LastSimulationTimeSeconds, 3);
            testCase.verifyEqual( ...
                chain.Modules{2}.LastSimulationTimeSeconds, 3);
        end

        function testRetainsUpdatedModuleStateBetweenApplications(testCase)
            module = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) value);
            chain = v2xsim.positioning.PositionErrorChain({module});
            positions = testCase.createPositions();

            chain = chain.apply( ...
                positions, testCase.createContext(positions, 0));
            chain = chain.apply( ...
                positions, testCase.createContext(positions, 1));

            testCase.verifyEqual( ...
                chain.Modules{1}.ApplicationCount, 2);
        end

        function testEmptyChainLeavesPositionsUnchanged(testCase)
            chain = v2xsim.positioning.PositionErrorChain();
            positions = testCase.createPositions();
            context = testCase.createContext(positions, 0);

            [chain, actualPositions] = chain.apply(positions, context);

            testCase.verifyEqual(actualPositions, positions);
            testCase.verifyEmpty(chain.Modules);
        end

        function testConstructorRejectsNonmoduleElement(testCase)
            constructor = @() ...
                v2xsim.positioning.PositionErrorChain({42});

            testCase.verifyError( ...
                constructor, ...
                "v2xsim:positioning:InvalidErrorModule");
        end
    end

    methods (Access = private)
        function positions = createPositions(~)
            positions = table( ...
                [10; 20], [30; 40], ...
                VariableNames=["X", "Y"], ...
                RowNames=["vehicle-1", "vehicle-2"]);
        end

        function positions = addToX(~, positions, offset)
            positions.X = positions.X + offset;
        end

        function positions = multiplyX(~, positions, multiplier)
            positions.X = positions.X .* multiplier;
        end

        function context = createContext( ...
                ~, actualPositions, simulationTimeSeconds)
            context = v2xsim.positioning.PositionErrorContext( ...
                actualPositions, simulationTimeSeconds, 0.1, 10);
        end
    end
end
