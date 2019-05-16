classdef pidAlgorithm < handle
    properties
        gains
        ix
        last_t
        last_x
    end
    
    methods
        function obj = pidAlgorithm(gains)
            obj.gains = gains;
            obj.ix = 0.0;
            obj.last_t = 0.0;
            obj.last_x = [];
        end
        
        function out = compute(obj, t, x, dx)
            dt = t - obj.last_t;
            obj.ix = obj.ix + dt * x;
            
%             if ~exist('dx', 'var')
%                 if isempty(obj.last_x)
%                     dx = 0;
%                 else
%                     dx = (x - obj.last_x) / dt;
%                 end
%             end
            
            out = double(obj.gains(1)*x + obj.gains(2)*obj.ix + obj.gains(3)*dx);
            
            obj.last_t = t;
            obj.last_x = x;
        end
    end
end

