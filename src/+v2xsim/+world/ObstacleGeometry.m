classdef (Abstract, HandleCompatible) ObstacleGeometry
    %OBSTACLEGEOMETRY Representation-neutral physical-obstacle contract.
    %   Concrete raster, polygon, or externally sourced geometries can be
    %   composed into World without putting propagation behavior in World.

    methods (Abstract)
        value = hasObstacles(obj)
    end
end
