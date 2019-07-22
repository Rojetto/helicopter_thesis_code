classdef FeedbackLinearization < HeliController
    %FeedbackLinearization Controller
    properties (Access = private)
        c
    end

    properties (Nontunable)
        %ke k Elevation
        ke = [20, 6]
        %kl k Travel
        kl = [1, 0.8]
        %kp k Pitch
        kp = [150, 20]
    end
    
    methods
        function obj = FeedbackLinearization()
            obj.c = Constants();
        end

        function initialize(obj)
        end
        
        function [u, debug_out] = control(obj, t, x)
            phi = x(1);
            eps = x(2);
            lamb = x(3);
            dphi = x(4);
            deps = x(5);
            dlamb = x(6);
            
            traj_eval = eval_trajectory(obj.trajectory, t);
            e_traj = traj_eval.eps;
            lambda_traj = traj_eval.lamb;

            v1 = e_traj(3) - obj.ke(2) * (deps - e_traj(2)) - obj.ke(1) * (eps - e_traj(1));
            v2 = lambda_traj(3) - obj.kl(2) * (dlamb - lambda_traj(2)) - obj.kl(1) * (lamb - lambda_traj(1));
            
            m_h = obj.c.m_h;
            m_c = obj.c.m_c;
            l_h = obj.c.l_h;
            l_c = obj.c.l_c;
            l_p = obj.c.l_p;
            d_h = obj.c.d_h;
            d_c = obj.c.d_c;
            
            mup = obj.c.mup;
            mue = obj.c.mue;
            mul = obj.c.mul;
            
            p_phi_1 = m_h*(l_p^2+d_h^2);
            p_eps_1 = m_c*(l_c^2+d_c^2) + m_h*(l_h^2+d_h^2) + m_h*sin(phi)^2*(l_p^2-d_h^2);
            p_lamb_1 = -d_c^2*m_c*cos(eps)^2-d_c*l_c*m_c*sin(2*eps)-d_h^2*m_h*cos(eps)^2*cos(phi)^2+d_h^2*m_h - d_h*l_h*m_h/2*(sin(2*eps-phi)+sin(2*eps+phi))+l_c^2*m_c*cos(eps)^2+l_h^2*m_h*cos(eps)^2+l_p^2*m_h*cos(eps)^2*cos(phi)^2-l_p^2*m_h*cos(eps)^2+l_p^2*m_h;
            
            p_phi_2 = - g*d_h*m_h*cos(eps);
            p_eps_2 = g*(d_c*m_c - d_h*m_h*cos(phi));
            p_eps_3 = g*(l_h*m_h - m_c*l_c);
            
            u1v = 1/l_h * (p_eps_1*v1 + p_eps_2*sin(eps)+p_eps_3*cos(eps)+mue*deps);
            u2v = 1/l_h * (p_lamb_1*v2 + mul*dlamb);

            Fs = sqrt(u1v^2 + u2v^2)*sign(u1v);

            phi_d = atan(u2v / u1v);

            dpd = 0;
            ddpd = 0;

            v3 = ddpd - obj.kp(2) * (dphi - dpd) - obj.kp(1) * (phi - phi_d);

            Fd = 1/l_p * (p_phi_1*v3 + p_phi_2 * sin(phi) + mup*dphi);

            Ff = (Fs + Fd) / 2;
            Fb = (Fs - Fd) / 2;
            
            uf = Fr_inv(Ff, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);
            ub = Fr_inv(Fb, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);

            u = [uf; ub];
            debug_out = [phi_d, u1v, u2v];
        end
    end
    
    methods (Access=protected)
        function out = getDebugOutputSize(~)
            out = 3;
        end
    end
end

