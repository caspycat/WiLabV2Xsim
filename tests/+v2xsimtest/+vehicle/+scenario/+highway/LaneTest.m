classdef LaneTest < matlab.unittest.TestCase
    %LANETEST Tests the minimal parametric lane contract.

    methods (Test)
        function testStoresLaneWidth(testCase)
            lane = testCase.createLane();

            testCase.verifyEqual(lane.WidthMeters, 3.5);
        end

        function testEvaluatesStartInteriorAndEndPositions(testCase)
            lane = testCase.createLane();
            progress = [0; 0.25; 1];

            actualPositions = lane.evaluate(progress);

            expectedPositions = [ ...
                0, 10; ...
                25, 15; ...
                100, 30];
            testCase.verifyEqual(actualPositions, expectedPositions);
        end

        function testAllowsEmptyProgressVector(testCase)
            lane = testCase.createLane();

            actualPositions = lane.evaluate(zeros(0, 1));

            testCase.verifyEqual(actualPositions, zeros(0, 2));
        end

        function testRejectsProgressOutsideLane(testCase)
            lane = testCase.createLane();

            testCase.verifyError( ...
                @() lane.evaluate(-0.01), ...
                "v2xsim:scenario:highway:ProgressOutsideLane");
            testCase.verifyError( ...
                @() lane.evaluate(1.01), ...
                "v2xsim:scenario:highway:ProgressOutsideLane");
        end

        function testRejectsNonfiniteProgress(testCase)
            lane = testCase.createLane();

            testCase.verifyFunctionThrows(@() lane.evaluate(NaN));
            testCase.verifyFunctionThrows(@() lane.evaluate(Inf));
        end

        function testRejectsWrongEvaluationShape(testCase)
            lane = ...
                v2xsimtest.vehicle.scenario.highway.fixture.LaneStub( ...
                    3.5, @(progress) zeros(numel(progress), 1));

            testCase.verifyError( ...
                @() lane.evaluate([0; 1]), ...
                "v2xsim:scenario:highway:InvalidLaneEvaluation");
        end

        function testRejectsNonfiniteEvaluation(testCase)
            lane = ...
                v2xsimtest.vehicle.scenario.highway.fixture.LaneStub( ...
                    3.5, @(progress) ...
                        Inf(numel(progress), 2));

            testCase.verifyError( ...
                @() lane.evaluate(0.5), ...
                "v2xsim:scenario:highway:InvalidLaneEvaluation");
        end

        function testRejectsNonpositiveWidth(testCase)
            constructor = @() ...
                v2xsimtest.vehicle.scenario.highway.fixture.LaneStub( ...
                    0, @(progress) zeros(numel(progress), 2));

            testCase.verifyFunctionThrows(constructor);
        end
    end

    methods (Access = private)
        function lane = createLane(~)
            lane = ...
                v2xsimtest.vehicle.scenario.highway.fixture.LaneStub( ...
                    3.5, ...
                    @(progress) [ ...
                        100 .* progress, ...
                        10 + 20 .* progress]);
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
