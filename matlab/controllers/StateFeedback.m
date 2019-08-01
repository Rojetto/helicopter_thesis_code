classdef StateFeedback < HeliController
    properties (Constant)
        n = 6
    end
    
    properties (Nontunable)
        K = zeros(2, 6);
    end
    
    methods
        function obj = StateFeedback()
        end
        
        function [u, debug_out] = control(obj, t, x_in)
            n = obj.n;
            x = reshape(x_in(1:n), n, 1);
            traj_eval = eval_trajectory(obj.trajectory, t);
            
            if n == 8
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
            else %n == 6
                x_d = [
                    traj_eval.phi(1); 
                    traj_eval.eps(1); 
                    traj_eval.lamb(1); 
                    traj_eval.phi(2); 
                    traj_eval.eps(2); 
                    traj_eval.lamb(2);
                ];
            end
            u_d = [traj_eval.vf(1); traj_eval.vb(1)];
            
            u = u_d - obj.K * (x - x_d);
            debug_out = [];
        end
        
        function initialize(obj)
        end
    end
end
