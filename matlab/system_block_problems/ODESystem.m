classdef ODESystem < matlab.System & matlab.system.mixin.Propagates
    properties (DiscreteState)
        x
    end
    
    methods (Access = protected)
        function y = stepImpl(obj, u)
            h = 0.1; % step width
            [~, xs] = ode45(@(t_, x_) ODESystem.sys_f(x_, u), [0, h/2, h], obj.x);
            obj.x = xs(end); % new state at t+h
            y = ODESystem.sys_h(obj.x); % output function
        end
        
        function resetImpl(obj)
            obj.x = 0;
        end
        
        function [out1] = isOutputFixedSizeImpl(~)
            out1 = true;
        end
        
        function [out1] = isOutputComplexImpl(~)
            out1 = false;
        end
        
        function [out1] = getOutputSizeImpl(~)
            out1 = 1;
        end
        
        function [out1] = getOutputDataTypeImpl(~)
            out1 = 'double';
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, prop_name)
            if strcmp(prop_name, 'x')
                sz = [1, 1];
                dt = 'double';
                cp = false;
            end
        end
    end
    
    methods (Static)
        function dx = sys_f(x, u)
            dx = u - x;
        end
        
        function y = sys_h(x)
            y = x^2;
        end
    end
end

