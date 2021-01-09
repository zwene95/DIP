classdef EKF
    %EKF Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        States;
        Controls;
        Measurements;
        ProcessNoise;
        MeasurementNoise;
        Q;
        R;
        StepTime;
    end
    
    properties (Dependent)
        ResultsEKF;
    end
    
    methods
        
        function ret = get.ResultsEKF(obj)
            % EKF Tracking
            
            ret = 0;
        end        
    end    
    
    methods (Access=protected)
        function ret = StateFcn(obj,dt)
        end
        function ret = MeasFcn(obj,dt)
        end
        function ret = StateJac_x(obj,dt)
        end
        function ret = StateJac_w(obj,dt)
        end
        function ret = MeasJac_x(obj,dt)
        end
    end
end

