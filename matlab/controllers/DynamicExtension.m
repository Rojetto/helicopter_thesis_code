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
        
        b1 = 100;
        b2 = 1;
        
        K
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
        use_weird_observer = true;
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
            debug_out = zeros(44, 1);
            
            %% Alternative state contains Fs and dFs in x7 and x8
            x_alt = zeros(8, 1);
            x_alt(1:6) = x(1:6);
            x_alt(7:8) = obj.x78;
            
            %% Compute coordinate transformation, and two values for linearized system
            if coder.target('MATLAB')
                [phi, gamma, lambda] = c_calcPhiGammaLambda_mex(x_alt);
            else
                [phi, gamma, lambda] = c_calcPhiGammaLambda(x_alt);
            end
            
            debug_out(37:44) = phi;
            
            %% Evaluate trajectory
            
            traj_eval = eval_trajectory(obj.trajectory, t);
            eps_traj = traj_eval.eps;
            lamb_traj = traj_eval.lamb;
            
            if obj.use_high_gain_observer || obj.use_weird_observer
                z_tilde = zeros(8, 1);
                z_tilde(1:4) = obj.z_est(1:4) - eps_traj(1:4)';
                z_tilde(5:8) = obj.z_est(5:8) - lamb_traj(1:4)';
            else
                z_tilde = zeros(8, 1);
                z_tilde(1:4) = phi(1:4) - eps_traj(1:4)';
                z_tilde(5:8) = phi(5:8) - lamb_traj(1:4)';
            end
            
            %% Linearization and stabilizing feedback
            if obj.smc
                %% SMC
                v = zeros(2, 1);
                sigma1 = sum(obj.k_eps' .* z_tilde(1:4));
                v(1) = eps_traj(4) - sum(obj.k_eps(1:3)' .* z_tilde(2:4)) - obj.a1 * tanh(obj.b1 * sigma1);
                
                sigma2 = sum(obj.k_lamb' .* z_tilde(5:8));
                v(2) = lamb_traj(4) - sum(obj.k_lamb(1:3)' .* z_tilde(6:8)) - obj.a2 * tanh(obj.b2 * sigma2);
                
                u_alt = lambda \ (v - gamma);
                
                ddFs = u_alt(1);
                Fd = u_alt(2);
                
                debug_out(25) = sigma1;
                debug_out(26) = sigma2;
                debug_out(27:28) = v;
                debug_out(29:30) = u_alt;
                debug_out(31) = -gamma(1);
                debug_out(32) = - sum(obj.k_eps(1:3)' .* z_tilde(2:4));
                debug_out(33) = - obj.a1 * sign(sigma1);
                debug_out(34:36) = z_tilde(2:4);
            else
                %% No SMC
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
                debug_out(1:8) = x_alt;
                debug_out(9:16) = obj.x_est;
                debug_out(17:24) = obj.z_est;
                
                u_obs = [ddFs; Fd];
                h = obj.step_width;

                ode_params = zeros(26, 1);
                ode_params(1:2) = x_alt([2, 3]);
                ode_params(3:10) = obj.x_est;
                ode_params(11:26) = obj.K;

                obj.z_est = ode_step(@DynamicExtension.high_gain_ode_rhs, obj.z_est, u_obs, h, ode_params);
                
                obj.x_est = phi_inv(obj.z_est, obj.x_est);
            elseif obj.use_weird_observer
                debug_out(1:8) = x_alt;
                debug_out(9:16) = obj.x_est;
                debug_out(17:24) = obj.z_est;
                
                u_obs = [ddFs; Fd];
                h = obj.step_width;

                ode_params = zeros(53, 1);
                ode_params(1:5) = x_alt([1, 2, 3, 7, 8]); % y
                ode_params(6:13) = obj.x_est;
                ode_params(14:53) = obj.K;

                obj.z_est = ode_step(@DynamicExtension.weird_ode_rhs, obj.z_est, u_obs, h, ode_params);
                
                obj.x_est = phi_inv(obj.z_est, obj.x_est);
            end
            
            %% Controller output
            u = [uf; ub];
%             if t > 60
%                 u = [1; 1];
%             else
%                 u = [0; 0];
%             end
        end
    end
    
    methods (Access = protected)
        function out = getDebugOutputSize(~)
            out = 44;
        end
    end
    
    methods (Static)
        function dx78 = dyn_ext_rhs(x78, ddFs, ~)
            dx78 = zeros(2, 1);
            dx78(1) = x78(2);
            dx78(2) = ddFs;
        end
        
        function dz_est = high_gain_ode_rhs(z_est, u, params)
            y = params(1:2);
            % WARNING, only works when z_est = Phi(x_est), needs to be
            % ensured externally
            x_est = params(3:10);
            K = zeros(8, 2);
            K(:) = params(11:26);

            %% Calculate Gamma and Lambda from state in orig coords
            if coder.target('MATLAB')
                [~, gamma, lambda] = c_calcPhiGammaLambda_mex(x_est);
            else
                [~, gamma, lambda] = c_calcPhiGammaLambda(x_est);
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
        
        function dz_est = weird_ode_rhs(z_est, u, params)
            y = params(1:5);
            % WARNING, only works when z_est = Phi(x_est), needs to be
            % ensured externally
            x_est = params(6:13);
            K = zeros(8, 5);
            K(:) = params(14:53);

            %% Calculate Gamma and Lambda from state in orig coords
            if coder.target('MATLAB')
                [~, gamma, lambda] = c_calcPhiGammaLambda_mex(x_est);
            else
                [~, gamma, lambda] = c_calcPhiGammaLambda(x_est);
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
            y_est = x_est([1, 2, 3, 7, 8]);
            dz_est = dz_est + K*(y - y_est);
        end
    end
end

