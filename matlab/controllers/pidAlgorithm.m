classdef pidAlgorithm < handle
    properties
        gains
        ix
        last_t
        has_last_x
        last_x
    end
    
    methods
        function obj = pidAlgorithm(gains)
            obj.gains = gains;
            obj.ix = 0.0;
            obj.last_t = 0.0;
            obj.has_last_x = false;
            obj.last_x = 0.0;
        end
        
        function reset(obj)
            obj.ix = 0.0;
            obj.last_t = 0.0;
            obj.has_last_x = false;
            obj.last_x = 0.0;
        end
        
        function out = compute(obj, t, x, dx)
            dt = t - obj.last_t;
            obj.ix = obj.ix + dt * x;
            
            out = double(obj.gains(1)*x + obj.gains(2)*obj.ix + obj.gains(3)*dx);
            
            obj.last_t = t;
            obj.last_x = x;
            obj.has_last_x = true;
        end
        
        function out = compute_fd(obj, t, x)
            dt = t - obj.last_t;
            
            if ~obj.has_last_x
                dx = 0;
            else
                dx = (x - obj.last_x) / dt;
            end
            
            out = obj.compute(t, x, dx);
        end
    end
end

