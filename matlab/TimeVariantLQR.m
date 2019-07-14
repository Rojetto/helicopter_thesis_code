classdef TimeVariantLQR < HeliController
    properties (Access=private)
        Q
        R
        R_inv
        S
        riccati_tau
        riccati_P_triu_tau
    end
    
    properties
        Q_diag = [3, 50, 20, 2, 1, 5, 1, 1]
        R_diag = [1, 1]
        S_diag = [3, 50, 20, 2, 1, 5, 1, 1]
        riccati_step_width = 0.1
    end
    
    properties (Constant)
        riccati_samples = 200
    end
    
    methods
        function obj = TimeVariantLQR()
            obj.Q = zeros(8);
            obj.R = zeros(2);
            obj.R_inv = zeros(2);
            obj.S = zeros(8);
            
            obj.riccati_tau = zeros(1000, 1);
            obj.riccati_P_triu_tau = zeros(1000, 36);
        end
        
        function initialize(obj)
            obj.Q = diag(obj.Q_diag);
            obj.R = diag(obj.R_diag);
            obj.R_inv = inv(obj.R);
            obj.S = diag(obj.S_diag);
            
            obj.solve_riccati()
        end
        
        function out = control(obj, t, x_in)
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
            
            tau = te - t;
            P_triu = interp1(obj.riccati_tau, obj.riccati_P_triu_tau, tau);
            P = TimeVariantLQR.triu_to_full(P_triu);
            [~, B] = TimeVariantLQR.get_SS_at_time(obj.trajectory, t);
            
            u_diff = - obj.R_inv * B' * P * x_diff;
            
            out = u_d + u_diff;
            
            if t == te
                out = u_d;
            end
        end
        
        function solve_riccati(obj)
            tau_e = obj.trajectory.t(end);
            %n_taus = floor(tau_e/obj.riccati_step_width + 1);
            n_taus = obj.riccati_samples;
            taus = linspace(0, tau_e, n_taus);
            
            P_triu_0 = TimeVariantLQR.full_to_triu(obj.S);
            
            TimeVariantLQR.riccati_rhs(0, P_triu_0, obj.trajectory, obj.R_inv, obj.Q);
            [~, P_trius] = ode45(@TimeVariantLQR.riccati_rhs, taus, P_triu_0);
            
            obj.riccati_tau(1:n_taus) = taus;
            obj.riccati_P_triu_tau(1:n_taus, :) = P_trius;
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
        
        function d_P_triu = riccati_rhs(tau, P_triu, trajectory_, R_inv_, Q_)
            persistent trajectory R_inv Q
            
            if isempty(trajectory)
                trajectory = struct('t', 0, 'phi', [0 0 0], 'eps', [0 0 0 0 0], 'lamb', [0 0 0 0 0], 'vf', 0, 'vb', 0);
                R_inv = zeros(2);
                Q = zeros(8);
            end
            
            if nargin==5
                trajectory = trajectory_;
                R_inv = R_inv_;
                Q = Q_;
            end
            
            te = trajectory.t(end);
            [A, B] = TimeVariantLQR.get_SS_at_time(trajectory, te - tau);
            P = TimeVariantLQR.triu_to_full(P_triu);
            
            d_P = - P * B * R_inv * B' * P' + P * A + A' * P + Q;
            d_P_triu = TimeVariantLQR.full_to_triu(d_P);
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
            
            A = compute_A_full(x1, x2, x3, x4, x5, x6, x7, x8, u1, u2);
            B = compute_B_full(x1, x2, x3, x4, x5, x6, x7, x8, u1, u2);
        end
    end
end
