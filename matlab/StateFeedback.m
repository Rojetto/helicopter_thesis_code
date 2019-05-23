classdef StateFeedback < HeliController
    properties (Access = private)
        trajectory
    end
    
    properties (Nontunable)
        K = zeros(2, 6);
    end
    
    methods
        function obj = StateFeedback()
            obj.trajectory = Trajectory([], [], [], [], [], []);
        end
        
        function u = control(obj, t, x)
            x = reshape(x(1:6), 6, 1);
            traj_eval = obj.trajectory.eval(t);
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
        end
        
        function initialize(obj, trajectory)
            obj.trajectory = trajectory;
        end
    end
end
