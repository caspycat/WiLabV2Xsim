classdef EmptyObstacleGeometry < v2xsim.world.ObstacleGeometry
    %EMPTYOBSTACLEGEOMETRY Explicit absence of physical obstacles.

    methods
        function value = hasObstacles(~)
            value = false;
        end
    end
end
