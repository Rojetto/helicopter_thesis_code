classdef StateFeedback < HeliController    
    properties (Nontunable)
        K = zeros(2, 8);
    end
    
    methods
        function obj = StateFeedback()
        end
        
        function [u, debug_out] = control(obj, t, x_in)
            x = reshape(x_in(1:8), 8, 1);
            traj_eval = eval_trajectory(obj.trajectory, t);
            x_d = [
                traj_eval.phi(1); 
                traj_eval.eps(1); 
                traj_eval.lamb(1); 
                traj_eval.phi(2); 
                traj_eval.eps(2); 
                traj_eval.lamb(2);
                traj_eval.vf(1);
                traj_eval.vb(1);
            ];
            u_d = [traj_eval.vf(1); traj_eval.vb(1)];
            
            u = u_d - obj.K * (x - x_d);
            debug_out = [];
        end
        
        function initialize(obj)
        end
    end
end
