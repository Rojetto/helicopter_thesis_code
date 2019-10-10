classdef StateFeedback < HeliController    
    properties
        K = zeros(2, 6);
    end
    
    methods
        function obj = StateFeedback()
        end
        
        function [u, debug_out] = control(obj, t, x_in)
            n = 6;
            x = reshape(x_in(1:n), n, 1);
            traj_eval = eval_trajectory(obj.trajectory, t);
            
            x_d = [
                traj_eval.phi(1); 
                traj_eval.eps(1); 
                traj_eval.lamb(1); 
                traj_eval.phi(2); 
                traj_eval.eps(2); 
                traj_eval.lamb(2);
            ];
            u_d = [traj_eval.vf(1); traj_eval.vb(1)];
            
            u = u_d - obj.K * (x - x_d);
            
            T = [0.5, 0.5; 0.5, -0.5];
            K_star = T * obj.K;
            k_eps = K_star(1, 2);
            k_deps = K_star(1, 5);
            x_diff = x - x_d;
            u_diff = - K_star * x_diff;
            debug_out = [-k_eps * x_diff(2); -k_deps * x_diff(5); u_diff(1)];
        end
        
        function initialize(obj)
        end
    end
    
    methods (Access = protected)
        function out = getDebugOutputSize(~)
            out = 3;
        end
    end
end
