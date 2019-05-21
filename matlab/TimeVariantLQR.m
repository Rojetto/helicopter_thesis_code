classdef TimeVariantLQR < HeliController
    properties (Access=private)
        Q
        R
        R_inv
        S
        ref_t
        ref_u
        ref_x
        riccati_tau
        riccati_P_triu_tau
    end
    
    properties (Nontunable)
        Q_diag = [2, 10, 4, 0.2, 0.2, 0.1]
        R_diag = [0.1, 0.1]
        S_diag = [1, 1, 1, 0.1, 0.1, 0.1]
        riccati_step_width = 0.1
    end
    
    methods
        function obj = TimeVariantLQR()
        end
        
        function initialize(obj)
            obj.Q = diag(obj.Q_diag);
            obj.R = diag(obj.R_diag);
            obj.R_inv = inv(obj.R);
            obj.S = diag(obj.S_diag);
            
            mat_content = coder.load('move1.mat');
            obj.ref_t = mat_content.t;
            obj.ref_u = mat_content.u;
            obj.ref_x = mat_content.x;
            
            obj.solve_riccati()
        end
        
        function out = control(obj, t, x, elevation_traj, travel_traj)
            x = reshape(x(1:6), 6, 1);
            te = obj.ref_t(end);
            if t > te
                t = te;
            end
            u_d = interp1(obj.ref_t, obj.ref_u, t)';
            x_d = interp1(obj.ref_t, obj.ref_x, t)';
            x_diff = x - x_d;
            
            tau = te - t;
            P_triu = interp1(obj.riccati_tau, obj.riccati_P_triu_tau, tau);
            P = TimeVariantLQR.triu_to_full(P_triu);
            [~, B] = obj.get_SS_at_time(t);
            
            u_diff = - obj.R_inv * B' * P * x_diff;
            
            out = u_d + u_diff;
        end
        
        function solve_riccati(obj)
            obj.riccati_tau = 0:obj.riccati_step_width:obj.ref_t(end);
            tau_e = obj.ref_t(end);
            P_triu_0 = TimeVariantLQR.full_to_triu(obj.S);
            
            [ode_tau, P_triu] = ode45(@(tau, P_triu) TimeVariantLQR.riccati_rhs(obj, tau, P_triu), [0, tau_e], P_triu_0);
            obj.riccati_tau = ode_tau;
            obj.riccati_P_triu_tau = P_triu;
        end
        
        function [A, B] = get_SS_at_time(obj, t)
            u = interp1(obj.ref_t, obj.ref_u, t);
            if t > obj.ref_t(end)
                u = obj.ref_u(end,:);
            end
            
            x = interp1(obj.ref_t, obj.ref_x, t);
            if t > obj.ref_t(end)
                x = obj.ref_x(end,:);
            end
            
            x1 = x(1);
            x2 = x(2);
            x3 = x(3);
            x4 = x(4);
            x5 = x(5);
            x6 = x(6);
            
            u1 = u(1);
            u2 = u(2);
            
            A = compute_A(x1, x2, x3, x4, x5, x6, u1, u2);
            B = compute_B(x1, x2, x3, x4, x5, x6, u1, u2);
        end
    end
    
    methods (Static)
        function out = full_to_triu(full_mat)
            indices = logical(triu(ones(6)));
            
            out = full_mat(indices);
        end
        
        function out = triu_to_full(triu_mat)
            indices = logical(triu(ones(6)));
            
            out = zeros(6);
            out(indices) = triu_mat;
            only_lower = out';
            out(indices') = only_lower(indices');
        end
        
        function d_P_triu = riccati_rhs(obj, tau, P_triu)
            te = obj.ref_t(end);
            [A, B] = obj.get_SS_at_time(te - tau);
            P = TimeVariantLQR.triu_to_full(P_triu);
            
            d_P = - P * B * obj.R_inv * B' * P' + P * A + A' * P + obj.Q;
            d_P_triu = TimeVariantLQR.full_to_triu(d_P);
        end
    end
end
