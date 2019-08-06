classdef HighGainObserver < matlab.System ...
        & matlab.system.mixin.Propagates
    % Simulates the nonlinear system with dynamic extension of Fs input using automatic differentiation.

    properties
        %z0 Initial state in linearized coordinates
        z0 = [-0.349065850398866;0;0.283870645721701;-0.0255697151663404;0;0;0;0]
        %x0 Initial state in original coordinates
        x0 = [0;-20/180*pi;0;0;0;0;1;0]
        step_width = 0.002
        
        K = [8 0; 24 0; 32 0; 16 0; 0 8; 0 24; 0 32; 0 16]
    end

    properties (DiscreteState)
        z
        x
    end

    methods (Access = protected)
        function setupImpl(obj, eps, lamb, ddFs, Fd)
            if coder.target('MATLAB')
                c_buildTapes_mex();
            else
                c_buildTapes();
            end
        end

        function z = stepImpl(obj, eps, lamb, ddFs, Fd)
            u = [ddFs; Fd];
            h = obj.step_width;
            
            ode_params = zeros(26, 1);
            ode_params(1:2) = [eps; lamb];
            ode_params(3:10) = obj.x;
            ode_params(11:26) = obj.K;
            
            z = ode_step(@HighGainObserver.ode_rhs, obj.z, u, h, ode_params);
            obj.z = z;
            
            obj.x = phi_inv(z, obj.x);
        end

        function resetImpl(obj)
            obj.z = obj.z0;
            obj.x = obj.x0;
        end
        
        function [out1] = isOutputFixedSizeImpl(~)
            out1 = true;
        end
        
        function [out1] = isOutputComplexImpl(~)
            out1 = false;
        end
        
        function [out1] = getOutputSizeImpl(~)
            out1 = 8;
        end
        
        function [out1] = getOutputDataTypeImpl(~)
            out1 = 'double';
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, prop_name)
            if strcmp(prop_name, 'z')
                sz = [8, 1];
                dt = 'double';
                cp = false;
            elseif strcmp(prop_name, 'x')
                sz = [8, 1];
                dt = 'double';
                cp = false;
            end
        end
    end
    
    methods (Static)
        function dz = ode_rhs(z, u, params)
            y = params(1:2);
            last_x = params(3:10);
            K = zeros(8, 2);
            K(:) = params(11:26);
            
            %% Calculate state in original coordinates
            x = phi_inv(z, last_x);

            %% Calculate Gamma and Lambda from state in orig coords
            if coder.target('MATLAB')
                gamma = c_calcGamma_mex(x);
                lambda = c_calcLambda_mex(x);
            else
                gamma = c_calcGamma(x);
                lambda = c_calcLambda(x);
            end
            %% Calculate derivative of linearized state
            dz = zeros(8, 1);

            dz(1) = z(2);
            dz(2) = z(3);
            dz(3) = z(4);

            dz(5) = z(6);
            dz(6) = z(7);
            dz(7) = z(8);

            dz([4,8]) = gamma + lambda * u;
            
            %% Add observer feedback
            y_est = z([1, 5]);
            dz = dz + K*(y - y_est);
        end
    end
end
