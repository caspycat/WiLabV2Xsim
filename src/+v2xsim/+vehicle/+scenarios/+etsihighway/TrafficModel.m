classdef TrafficModel
    %TRAFFICMODEL ETSI freeway speed-density operating points.

    properties (SetAccess = immutable)
        AverageVehicleSpeedKilometersPerHour (1, 1) double
        TargetAverageHeadway (1, 1) double
    end

    enumeration
        HighSpeedLowDensity (250)
        MediumSpeedMediumDensity (140)
        LowSpeedHighDensity (70)
    end

    methods
        function obj = TrafficModel( ...
                averageVehicleSpeedKilometersPerHour)
            obj.AverageVehicleSpeedKilometersPerHour = ...
                averageVehicleSpeedKilometersPerHour;
            obj.TargetAverageHeadway = ...
                2.5 .* averageVehicleSpeedKilometersPerHour ./ 3.6;
        end
    end
end
