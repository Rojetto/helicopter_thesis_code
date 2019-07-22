classdef DynamicExtensionSim < matlab.System
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
        function setupImpl(obj, ddFs, Fb)
            c_buildTapes_mex();
        end

        function x = stepImpl(obj, ddFs, Fb)
            u = [ddFs; Fb];
            h = obj.step_width;
            
            [~, x_ode] = ode45(@(t_, x_) DynamicExtensionSim.ode_rhs(x_, u), [0; h/2; h], obj.x);
            x = x_ode(end,:)';
            obj.x = x;
        end

        function resetImpl(obj)
            obj.x = obj.x0;
        end
    end
    
    methods (Static)
        function dx = ode_rhs(x, u)
            gamma = c_calcGamma_mex(x);
            lambda = c_calcLambda_mex(x);
            
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
