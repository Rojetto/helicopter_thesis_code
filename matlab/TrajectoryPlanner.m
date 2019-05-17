classdef TrajectoryPlanner < handle
    properties
        ya
        yb
        k
        ta
        tb
        c
    end
    
    methods
        function obj = TrajectoryPlanner(ya, yb, ta, tb)
            obj.ya = ya;
            obj.yb = yb;
            obj.ta = ta;
            obj.tb = tb;
            
            k = numel(ya); % number of derivative constraints (including value)
            obj.k = k;
            A = zeros(2*k,2*k);
            A(1:k,:) = TrajectoryPlanner.t_matrix(k, ta);
            A(k+1:2*k,:) = TrajectoryPlanner.t_matrix(k, tb);
            b = zeros(2*k, 1);
            b(1:k) = ya;
            b(k+1:2*k) = yb;
            
            obj.c = A \ b;
        end
        
        function ys = eval(obj, ts)
            n = numel(ts);
            ys = zeros(obj.k, n);
            
            for i=1:n
                t = ts(i);
                
                if t < obj.ta
                    y = obj.ya;
                elseif t > obj.tb
                    y = obj.yb;
                else
                    T = TrajectoryPlanner.t_matrix(obj.k, t);
                    y = T * obj.c;
                end
                
                ys(:,i) = y;
            end
        end
    end
    
    methods (Static)
        function T = t_matrix(k, t)
            T = zeros(k, 2*k);
            for i=1:2*k
                T(1,i) = t^(i-1);
            end
            for i=2:k
                T(i,i:end) = T(i-1,i-1:end-1) .* (i-1:2*k-1);
            end
        end
    end
end

