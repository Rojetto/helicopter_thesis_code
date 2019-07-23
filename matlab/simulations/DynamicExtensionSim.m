classdef DynamicExtensionSim < matlab.System ...
        & matlab.system.mixin.Propagates
    % Simulates the nonlinear system with dynamic extension of Fs input using automatic differentiation.

    properties
        %z0 Initial state in linearized coordinates
        z0 = [0;0;-0.215515513016254;0.019412610514001;0;0;0;0]
        %x0 Initial state in original coordinates
        x0 = [0;0;0;0;0;0;1;0]
        step_width = 0.002
    end

    properties (DiscreteState)
        z
        x
    end

    methods (Access = protected)
        function setupImpl(obj, ddFs, Fd)
            if coder.target('MATLAB')
                c_buildTapes_mex();
            else
                c_buildTapes();
            end
        end

        function z = stepImpl(obj, ddFs, Fd)
            u = [ddFs; Fd];
            h = obj.step_width;
            
            z = ode_step(@DynamicExtensionSim.ode_rhs, obj.z, u, h, obj.x);
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
            %% Calculate state in original coordinates
            last_x = params;
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
        end
    end
end
