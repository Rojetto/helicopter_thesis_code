classdef DynamicExtensionSim < matlab.System ...
        & matlab.system.mixin.Propagates
    % Simulates the nonlinear system with dynamic extension of Fs input using automatic differentiation.

    properties
        %x0 Initial state
        x0 = zeros(8, 1)
        step_width = 0.002
    end

    properties (DiscreteState)
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

        function x = stepImpl(obj, ddFs, Fd)
            u = [ddFs; Fd];
            h = obj.step_width;
            
            x = ode_step(@DynamicExtensionSim.ode_rhs, obj.x, u, h);
            obj.x = x;
        end

        function resetImpl(obj)
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
            if strcmp(prop_name, 'x')
                sz = [8, 1];
                dt = 'double';
                cp = false;
            end
        end
    end
    
    methods (Static)
        function dx = ode_rhs(x, u)
            if coder.target('MATLAB')
                gamma = c_calcGamma_mex(x);
                lambda = c_calcLambda_mex(x);
            else
                gamma = c_calcGamma(x);
                lambda = c_calcLambda(x);
            end
            
            dx = zeros(8, 1);
            
            dx(1) = x(2);
            dx(2) = x(3);
            dx(3) = x(4);
            
            dx(5) = x(6);
            dx(6) = x(7);
            dx(7) = x(8);
            
            dx([4,8]) = gamma + lambda * u;
        end
    end
end
