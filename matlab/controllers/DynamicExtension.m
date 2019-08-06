classdef DynamicExtension < HeliController
    %Using feedback linearization on a system with dynamic extension.
    properties (Access = private)
        c
        
        x78
        z_est
        x_est
    end
    
    properties
        k_eps = [1, 4, 6, 4];
        
        k_lamb = [1, 4, 6, 4];
        
        a1 = 1;
        a2 = 1;
        
        K = [8 0; 24 0; 32 0; 16 0; 0 8; 0 24; 0 32; 0 16]
    end
    
    properties (Nontunable)
        x780 = [1; 0];
        z_est0 = [-0.349065850398866;0;0.283870645721701;-0.0255697151663404;0;0;0;0];
        x_est0 = [0;-20/180*pi;0;0;0;0;1;0];
        step_width = 0.002;
    end
    
    properties (Nontunable, Logical)
        smc = true;
        use_high_gain_observer = true;
    end
    
    methods
        function obj = DynamicExtension()
            obj.c = Constants();
            obj.x78 = zeros(2, 1);
            obj.z_est = zeros(8, 1);
            obj.x_est = zeros(8, 1);
        end
        
        function initialize(obj)
            if coder.target('MATLAB')
                c_buildTapes_mex();
            else
                c_buildTapes();
            end
            
            obj.x78 = obj.x780;
            obj.z_est = obj.z_est0;
            obj.x_est = obj.x_est0;
        end
        
        function [u, debug_out] = control(obj, t, x)
            debug_out = zeros(14, 1);
            
            %% Alternative state contains Fs and dFs in x7 and x8
            x_alt = zeros(8, 1);
            x_alt(1:6) = x(1:6);
            x_alt(7:8) = obj.x78;
            
            debug_out(1:2) = obj.x78;
            
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
            
            %% Evaluate trajectory
            
            traj_eval = eval_trajectory(obj.trajectory, t);
            eps_traj = traj_eval.eps;
            lamb_traj = traj_eval.lamb;
            
            if obj.use_high_gain_observer
                z_tilde = zeros(8, 1);
                z_tilde(1:4) = obj.z_est(1:4) - eps_traj(1:4)';
                z_tilde(5:8) = obj.z_est(5:8) - lamb_traj(1:4)';
            else
                z_tilde = zeros(8, 1);
                z_tilde(1:4) = phi(1:4) - eps_traj(1:4)';
                z_tilde(5:8) = phi(5:8) - lamb_traj(1:4)';
            end
            
            debug_out(3:10) = z_tilde;
            
            %% Linearization and stabilizing feedback
            if obj.smc
                %% SMC
                w = zeros(2, 1);
                sigma1 = sum(obj.k_eps' .* z_tilde(1:4));
                w(1) = -gamma(1) - sum(obj.k_eps(1:3)' .* z_tilde(2:4)) - obj.a1 * sign(sigma1);
                
                sigma2 = sum(obj.k_lamb' .* z_tilde(5:8));
                w(2) = -gamma(2) - sum(obj.k_lamb(1:3)' .* z_tilde(6:8)) - obj.a2 * sign(sigma2);
                
                debug_out(11) = sigma1;
                debug_out(12) = sigma2;
                
                u_alt = lambda \ w;
                
                ddFs = u_alt(1);
                Fd = u_alt(2);
                
                debug_out(13:14) = u_alt;
            else
                %% No SMC
                sum(obj.k_eps' .* z_tilde(1:4));
                v1 = eps_traj(5) - sum(obj.k_eps' .* z_tilde(1:4));

                v2 = lamb_traj(5) - sum(obj.k_lamb' .* z_tilde(5:8));

                v = [v1; v2];

                u_alt = lambda \ (v - gamma);

                ddFs = u_alt(1);
                Fd = u_alt(2);
            end
            
            %% Integrate ddFs to get Fs
            h = obj.step_width;
            
            obj.x78 = ode_step(@DynamicExtension.dyn_ext_rhs, obj.x78, ddFs, h, []);
            x_alt(7:8) = obj.x78;
            
            Fs = obj.x78(1);
            
            %% Convert forces to voltages            
            Ff = (Fs + Fd)/2;
            Fb = (Fs - Fd)/2;
            
            uf = Fr_inv(Ff, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);
            ub = Fr_inv(Fb, obj.c.p1, obj.c.q1, obj.c.p2, obj.c.q2);
            
            %% High Gain observer step to estimate z
            if obj.use_high_gain_observer
                u_obs = [ddFs; Fd];
                h = obj.step_width;

                ode_params = zeros(26, 1);
                ode_params(1:2) = x_alt([2, 3]);
                ode_params(3:10) = obj.x_est;
                ode_params(11:26) = obj.K;

                obj.z_est = ode_step(@DynamicExtension.obs_ode_rhs, obj.z_est, u_obs, h, ode_params);
                
                obj.x_est = phi_inv(obj.z_est, obj.x_est);
            end
            
            %% Controller output
            u = [uf; ub];
        end
    end
    
    methods (Access = protected)
        function out = getDebugOutputSize(~)
            out = 14;
        end
    end
    
    methods (Static)
        function dx78 = dyn_ext_rhs(x78, ddFs, ~)
            dx78 = zeros(2, 1);
            dx78(1) = x78(2);
            dx78(2) = ddFs;
        end
        
        function dz_est = obs_ode_rhs(z_est, u, params)
            y = params(1:2);
            last_x = params(3:10);
            K = zeros(8, 2);
            K(:) = params(11:26);
            
            %% Calculate state in original coordinates
            x = phi_inv(z_est, last_x);

            %% Calculate Gamma and Lambda from state in orig coords
            if coder.target('MATLAB')
                gamma = c_calcGamma_mex(x);
                lambda = c_calcLambda_mex(x);
            else
                gamma = c_calcGamma(x);
                lambda = c_calcLambda(x);
            end
            %% Calculate derivative of linearized state
            dz_est = zeros(8, 1);

            dz_est(1) = z_est(2);
            dz_est(2) = z_est(3);
            dz_est(3) = z_est(4);

            dz_est(5) = z_est(6);
            dz_est(6) = z_est(7);
            dz_est(7) = z_est(8);

            dz_est([4,8]) = gamma + lambda * u;
            
            %% Add observer feedback
            y_est = z_est([1, 5]);
            dz_est = dz_est + K*(y - y_est);
        end
    end
end

