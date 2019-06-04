classdef TimeVariantLQRI < HeliController
    properties (Access=private)
        Q
        R
        R_inv
        S
        riccati_tau
        riccati_P_triu_tau
        yi
        
        trajectory
    end
    
    properties (Nontunable)
        Q_diag = [3, 50, 20, 2, 1, 5, 1, 1, 0.1, 10, 10]
        R_diag = [1, 1]
        S_diag = [3, 50, 20, 2, 1, 5, 1, 1, 0.1, 10, 10 ]
        riccati_step_width = 0.1
        sim_step_width = 1/60
    end
    
    methods
        function obj = TimeVariantLQRI()
            obj.trajectory = Trajectory([], [], [], [], [], []);
            
            obj.Q = zeros(11);
            obj.R = zeros(2);
            obj.R_inv = zeros(2);
            obj.S = zeros(11);
            
            obj.riccati_tau = [];
            obj.riccati_P_triu_tau = [];
            
            obj.yi = zeros(3, 1);
        end
        
        function initialize(obj, trajectory)
            obj.trajectory = trajectory;
            
            obj.Q = diag(obj.Q_diag);
            obj.R = diag(obj.R_diag);
            obj.R_inv = inv(obj.R);
            obj.S = diag(obj.S_diag);
            
            obj.yi = zeros(3, 1);
            
            obj.solve_riccati()
        end
        
        function out = control(obj, t, x)
            x = reshape(x(1:8), 8, 1);
            te = obj.trajectory.t(end);
            if t > te
                t = te;
            end
            traj_eval = obj.trajectory.eval(t);
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
            x_diff = [x - x_d];
            obj.yi = obj.yi + x_diff(1:3)*obj.sim_step_width;
            
            x_bar = [x_diff; obj.yi];
            %x_bars = evalin('base', 'x_bars');
            %x_bars(end+1, :) = x_bar';
            %assignin('base', 'x_bars', x_bars);
            
            tau = te - t;
            P_triu = interp1(obj.riccati_tau, obj.riccati_P_triu_tau, tau);
            P = TimeVariantLQRI.triu_to_full(P_triu);
            [~, B] = obj.get_SS_at_time(t);
            
            u_diff = - obj.R_inv * B' * P * x_bar;
            
            out = u_d + u_diff;
            
            if t == te
                out = u_d;
            end
        end
        
        function solve_riccati(obj)
            tau_e = obj.trajectory.t(end);
            P_triu_0 = TimeVariantLQRI.full_to_triu(obj.S);
            
            [ode_tau, P_triu] = ode45(@(tau, P_triu) TimeVariantLQRI.riccati_rhs(obj, tau, P_triu), [0, tau_e], P_triu_0);
            obj.riccati_tau = ode_tau;
            obj.riccati_P_triu_tau = P_triu;
        end
        
        function [A, B] = get_SS_at_time(obj, t)
            traj_eval = obj.trajectory.eval(t);
            
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
    
    methods (Static)
        function out = full_to_triu(full_mat)
            indices = logical(triu(ones(11)));
            
            out = full_mat(indices);
        end
        
        function out = triu_to_full(triu_mat)
            indices = logical(triu(ones(11)));
            
            out = zeros(11);
            out(indices) = triu_mat;
            only_lower = out';
            out(indices') = only_lower(indices');
        end
        
        function d_P_triu = riccati_rhs(obj, tau, P_triu)
            te = obj.trajectory.t(end);
            [A, B] = obj.get_SS_at_time(te - tau);
            P = TimeVariantLQRI.triu_to_full(P_triu);
            
            d_P = - P * B * obj.R_inv * B' * P' + P * A + A' * P + obj.Q;
            d_P_triu = TimeVariantLQRI.full_to_triu(d_P);
        end
    end
end