classdef TimeVariantLQR_verify < handle
    properties (Access=private)
        Q
        R
        R_inv
        S
        riccati_tau
        riccati_P_triu_tau
    end
    
    properties
        Q_diag = [1,1]
        R_diag = [1]
        S_diag = [1,1]
        riccati_step_width = 0.01
        te = 2
    end
    
    methods
        function obj = TimeVariantLQR_verify()
            obj.Q = zeros(2);
            obj.R = zeros(1);
            obj.R_inv = zeros(1);
            obj.S = zeros(2);
            
            obj.riccati_tau = [];
            obj.riccati_P_triu_tau = [];
        end
        
        function initialize(obj)
            obj.Q = diag(obj.Q_diag);
            obj.R = diag(obj.R_diag);
            obj.R_inv = inv(obj.R);
            obj.S = diag(obj.S_diag);
            
            obj.solve_riccati()
        end
        
        function [u, p12, p22] = control(obj, t, x)
            x = reshape(x(1:2), 2, 1);
            
            if t > obj.te
                t = obj.te;
            end
            
            tau = obj.te - t;
            P_triu = interp1(obj.riccati_tau, obj.riccati_P_triu_tau, tau);
            P = TimeVariantLQR_verify.triu_to_full(P_triu);
            [~, B] = obj.get_SS_at_time(t);
            
            u = - obj.R_inv * B' * P * x;
            p12 = P(1,2);
            p22 = P(2,2);
        end
        
        function solve_riccati(obj)
            tau_e = obj.te;
            P_triu_0 = TimeVariantLQR_verify.full_to_triu(obj.S);
            
            [ode_tau, P_triu] = ode45(@(tau, P_triu) TimeVariantLQR_verify.riccati_rhs(obj, tau, P_triu), [0, tau_e], P_triu_0);
            obj.riccati_tau = ode_tau;
            obj.riccati_P_triu_tau = P_triu;
        end
        
        function [A, B] = get_SS_at_time(obj, t)
            A = [0, 1; 0, -1];
            B = [0; 1];
        end
    end
    
    methods (Static)
        function out = full_to_triu(full_mat)
            indices = logical(triu(ones(2)));
            
            out = full_mat(indices);
        end
        
        function out = triu_to_full(triu_mat)
            indices = logical(triu(ones(2)));
            
            out = zeros(2);
            out(indices) = triu_mat;
            only_lower = out';
            out(indices') = only_lower(indices');
        end
        
        function d_P_triu = riccati_rhs(obj, tau, P_triu)
            [A, B] = obj.get_SS_at_time(obj.te - tau);
            P = TimeVariantLQR_verify.triu_to_full(P_triu);
            
            d_P = - P * B * obj.R_inv * B' * P' + P * A + A' * P + obj.Q;
            d_P_triu = TimeVariantLQR_verify.full_to_triu(d_P);
        end
    end
end
