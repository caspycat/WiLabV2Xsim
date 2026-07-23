classdef PositionErrorChain < v2xsim.positioning.PositionErrorModule
    %POSITIONERRORCHAIN Applies position error modules in a fixed order.
    %   Modules are applied from left to right. Each module receives the
    %   apparent output of the preceding module and the same immutable
    %   context. An empty chain leaves positions unchanged.

    properties (SetAccess = private)
        Modules (1, :) cell = cell(1, 0)
    end

    methods
        function obj = PositionErrorChain(modules)
            %POSITIONERRORCHAIN Construct a chain from a row cell array.
            arguments (Input)
                modules (1, :) cell = cell(1, 0)
            end

            for moduleIndex = 1:numel(modules)
                module = modules{moduleIndex};
                if ~isa(module, ...
                        "v2xsim.positioning.PositionErrorModule") || ...
                        ~isscalar(module)
                    error( ...
                        "v2xsim:positioning:InvalidErrorModule", ...
                        "Chain element %d must be a scalar " + ...
                        "PositionErrorModule.", ...
                        moduleIndex);
                end
            end

            obj.Modules = modules;
        end
    end

    methods (Access = protected)
        function [obj, outputPositions] = doApply( ...
                obj, inputPositions, context)
            outputPositions = inputPositions;

            for moduleIndex = 1:numel(obj.Modules)
                module = obj.Modules{moduleIndex};
                [module, outputPositions] = module.apply( ...
                    outputPositions, context);
                obj.Modules{moduleIndex} = module;
            end
        end
    end
end
