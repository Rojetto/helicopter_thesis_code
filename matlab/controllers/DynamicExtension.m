classdef DynamicExtension < HeliController
    %Using feedback linearization on a system with dynamic extension.
    properties (Access = private)
        c
        
        x78
    end
    
    properties
        k_eps = [1, 4, 6, 4];
        
        k_lamb = [1, 4, 6, 4];
    end
    
    properties (Nontunable)
        x780 = [1; 0];
        step_width = 0.002;
    end
    
    methods
        function obj = DynamicExtension()
            obj.c = Constants();
            obj.x78 = zeros(2, 1);
        end

        function initialize(obj)
            if coder.target('MATLAB')
                c_buildTapes_mex();
            else
                c_buildTapes();
            end
            
            obj.x78 = obj.x780;
        end
        
        function [u, debug_out] = control(obj, t, x)
            %% Alternative state contains Fs and dFs in x7 and x8
            x_alt = zeros(8, 1);
            x_alt(1:6) = x(1:6);
            x_alt(7:8) = obj.x78;
            
            %% Compute coordinate transformation, and two values for linearized system
            if coder.target('MATLAB')
                phi = c_calcPhi_mex(x_alt);
                gamma = c_calcGamma_mex(x_alt);
                lambda = c_calcLambda_mex(x_alt);
            else
                phi = c_calcPhi(x_alt);
                gamma = c_calcGamma(x_alt);
                lambda = c_calcLambda(x_alt);
            end
            
            %% Compute stabilizing values for virtual inputs on linearized system
            eps = phi(1);
            d1eps = phi(2);
            d2eps = phi(3);
            d3eps = phi(4);
            
            lamb = phi(5);
            d1lamb = phi(6);
            d2lamb = phi(7);
            d3lamb = phi(8);
            
            traj_eval = eval_trajectory(obj.trajectory, t);
            eps_traj = traj_eval.eps;
            lamb_traj = traj_eval.lamb;
            
            v1 = traj_eval.eps(4) - obj.k_eps(4) * (d3eps - eps_traj(4))...
                                  - obj.k_eps(3) * (d2eps - eps_traj(3))...
                                  - obj.k_eps(2) * (d1eps - eps_traj(2))...
                                  - obj.k_eps(1) * (eps - eps_traj(1));
                              
            v2 = traj_eval.lamb(4) - obj.k_lamb(4) * (d3lamb - lamb_traj(4))...
                                   - obj.k_lamb(3) * (d2lamb - lamb_traj(3))...
                                   - obj.k_lamb(2) * (d1lamb - lamb_traj(2))...
                                   - obj.k_lamb(1) * (lamb - lamb_traj(1));
                               
            %% Linearizing feedback
            v = [v1; v2];
            
            u_alt = lambda \ (v - gamma);
            
            ddFs = u_alt(1);
            Fd = u_alt(2);
            
            %% Integrate ddFs to get Fs
            h = obj.step_width;
            
            obj.x78 = ode_step(@DynamicExtension.dyn_ext_rhs, obj.x78, ddFs, h, []);
            
            Fs = obj.x78(1);
            
            %% Convert forces to voltages            
            Ff = (Fs + Fd)/2;
            Fb = (Fs - Fd)/2;
            
            uf = Fr_inv(Ff, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);
            ub = Fr_inv(Fb, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);
            
            u = [uf; ub];
            debug_out = [];
        end
    end
    
    methods (Static)
        function dx78 = dyn_ext_rhs(x78, ddFs, ~)
            dx78 = zeros(2, 1);
            dx78(1) = x78(2);
            dx78(2) = ddFs;
        end
    end
end

