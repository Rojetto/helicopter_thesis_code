classdef TimeVariantLQRI < HeliController
    properties (Access=private)
        R_inv
        yi
    end
    
    properties (Nontunable)
        Q = diag([3, 50, 20, 2, 1, 5, 1, 1, 0.1, 10, 10])
        R = diag([1, 1])
        S = diag([3, 50, 20, 2, 1, 5, 1, 1, 0.1, 10, 10])
        
        riccati_tau
        riccati_P_triu_tau
        
        step_width = 0.002;
    end
    
    methods
        function obj = TimeVariantLQRI()
            obj.R_inv = zeros(2);
            
            obj.yi = zeros(3, 1);
        end
        
        function initialize(obj)
            obj.R_inv = inv(obj.R);
            
            obj.yi = zeros(3, 1);
        end
        
        function [out, debug_out] = control(obj, t, x_in)
            x = reshape(x_in(1:8), 8, 1);
            te = obj.trajectory.t(end);
            if t > te
                t = te;
            end
            traj_eval = eval_trajectory(obj.trajectory, t);
            u_d = [traj_eval.vf(1); traj_eval.vb(1)];
            x_d = [
                traj_eval.phi(1); 
                traj_eval.eps(1); 
                traj_eval.lamb(1); 
                traj_eval.phi(2); 
                traj_eval.eps(2); 
                traj_eval.lamb(2);
                traj_eval.vf(1);
                traj_eval.vb(1)
            ];
            x_diff = x - x_d;
            obj.yi = obj.yi + x_diff(1:3)*obj.step_width;
            
            x_bar = [x_diff; obj.yi];
            
            tau = te - t;
            P_triu = interp1(obj.riccati_tau, obj.riccati_P_triu_tau, tau);
            P = TimeVariantLQRI.triu_to_full(P_triu);
            [~, B] = TimeVariantLQRI.get_SS_at_time(obj.trajectory, t);
            
            u_diff = - obj.R_inv * B' * P * x_bar;
            
            out = u_d + u_diff;
            
            if t == te
                out = u_d;
            end
            
            debug_out = [];
        end
    end
    
    methods (Static)
        function out = full_to_triu(full_mat)
            n = size(full_mat, 1);
            indices = logical(triu(ones(n)));
            
            out = full_mat(indices);
        end
        
        function out = triu_to_full(triu_mat)
            k = length(triu_mat);
            n = (sqrt(1+8*k)-1)/2;
            indices = logical(triu(ones(n)));
            
            out = zeros(n);
            out(indices) = triu_mat;
            only_lower = out';
            out(indices') = only_lower(indices');
        end
        
        function d_P_triu = riccati_rhs(tau, P_triu, trajectory, R_inv, Q)
            te = trajectory.t(end);
            [A, B] = TimeVariantLQRI.get_SS_at_time(trajectory, te - tau);
            P = TimeVariantLQRI.triu_to_full(P_triu);
            
            d_P = - P * B * R_inv * B' * P' + P * A + A' * P + Q;
            d_P_triu = TimeVariantLQRI.full_to_triu(d_P);
        end
        
        function [A, B] = get_SS_at_time(trajectory, t)
            traj_eval = eval_trajectory(trajectory, t);
            
            x1 = traj_eval.phi(1);
            x2 = traj_eval.eps(1);
            x3 = traj_eval.lamb(1);
            x4 = traj_eval.phi(2);
            x5 = traj_eval.eps(2);
            x6 = traj_eval.lamb(2);
            x7 = traj_eval.vf(1);
            x8 = traj_eval.vb(1);
            
            u1 = traj_eval.vf(1);
            u2 = traj_eval.vb(1);
            
            A = zeros(11);
            A(1:8, 1:8) = compute_A_full(x1, x2, x3, x4, x5, x6, x7, x8, u1, u2);
            A(9:11, 1:3) = eye(3);
            B = zeros(11, 2);
            B(1:8,:) = compute_B_full(x1, x2, x3, x4, x5, x6, x7, x8, u1, u2);
        end
    end
end
