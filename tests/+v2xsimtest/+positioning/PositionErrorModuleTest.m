classdef PositionErrorModuleTest < matlab.unittest.TestCase
    %POSITIONERRORMODULETEST Tests the position error module contract.

    methods (Test)
        function testApplyReturnsTransformedPositionsAndUpdatedState( ...
                testCase)
            positions = testCase.createPositions();
            module = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) testCase.offsetPositions(value, [2, -3]));

            [module, actualPositions] = module.apply(positions, 1.25);

            expectedPositions = testCase.offsetPositions(positions, [2, -3]);
            testCase.verifyEqual(actualPositions, expectedPositions);
            testCase.verifyEqual(module.ApplicationCount, 1);
            testCase.verifyEqual(module.LastSimulationTimeSeconds, 1.25);
        end

        function testApplyRejectsInvalidInputSchema(testCase)
            positions = testCase.createPositions();
            positions.Z = zeros(height(positions), 1);
            module = testCase.createIdentityModule();

            testCase.verifyError( ...
                @() module.apply(positions, 0), ...
                "v2xsim:positioning:MustBe2DPositionTable");
        end

        function testApplyRejectsNonfiniteCoordinates(testCase)
            positions = testCase.createPositions();
            positions.X(1) = Inf;
            module = testCase.createIdentityModule();

            testCase.verifyError( ...
                @() module.apply(positions, 0), ...
                "v2xsim:positioning:MustBe2DPositionTable");
        end

        function testApplyRejectsMissingVehicleIdentities(testCase)
            positions = testCase.createPositions();
            positions.Properties.RowNames = {};
            module = testCase.createIdentityModule();

            testCase.verifyError( ...
                @() module.apply(positions, 0), ...
                "v2xsim:positioning:MustBe2DPositionTable");
        end

        function testApplyRejectsNegativeSimulationTime(testCase)
            positions = testCase.createPositions();
            module = testCase.createIdentityModule();

            testCase.verifyFunctionThrows( ...
                @() module.apply(positions, -0.1));
        end

        function testApplyRejectsInvalidOutputSchema(testCase)
            positions = testCase.createPositions();
            module = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) removevars(value, "Y"));

            testCase.verifyError( ...
                @() module.apply(positions, 0), ...
                "v2xsim:positioning:MustBe2DPositionTable");
        end

        function testApplyRejectsChangedVehicleSet(testCase)
            positions = testCase.createPositions();
            module = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) testCase.renameFirstVehicle(value));

            testCase.verifyError( ...
                @() module.apply(positions, 0), ...
                "v2xsim:positioning:VehicleSetChanged");
        end

        function testApplyAllowsVehicleRowReordering(testCase)
            positions = testCase.createPositions();
            module = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) flipud(value));

            [~, actualPositions] = module.apply(positions, 0);

            testCase.verifyEqual(actualPositions, flipud(positions));
        end

        function testApplyAllowsEmptyPositionSnapshot(testCase)
            positions = table( ...
                zeros(0, 1), zeros(0, 1), ...
                VariableNames=["X", "Y"]);
            module = testCase.createIdentityModule();

            [~, actualPositions] = module.apply(positions, 0);

            testCase.verifyEqual(actualPositions, positions);
        end
    end

    methods (Access = private)
        function module = createIdentityModule(~)
            module = ...
                v2xsimtest.positioning.fixture.PositionErrorModuleStub( ...
                    @(value, ~) value);
        end

        function positions = createPositions(~)
            positions = table( ...
                [10; 20], [30; 40], ...
                VariableNames=["X", "Y"], ...
                RowNames=["vehicle-1", "vehicle-2"]);
        end

        function positions = offsetPositions(~, positions, offset)
            positions.X = positions.X + offset(1);
            positions.Y = positions.Y + offset(2);
        end

        function positions = renameFirstVehicle(~, positions)
            rowNames = positions.Properties.RowNames;
            rowNames{1} = 'other-vehicle';
            positions.Properties.RowNames = rowNames;
        end

        function verifyFunctionThrows(testCase, functionHandle)
            didThrow = false;
            try
                functionHandle();
            catch
                didThrow = true;
            end

            testCase.verifyTrue(didThrow, ...
                "Expected the function to throw an exception.");
        end
    end
end
