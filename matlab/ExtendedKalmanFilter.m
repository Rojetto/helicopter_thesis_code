classdef ExtendedKalmanFilter < matlab.System ...
        & matlab.system.mixin.Propagates
    properties (Access = private)
        N
        W
    end
    
    properties (Nontunable)
        x0 = [0; deg2rad(-29); 0; 0; 0; 0; 0; 0]
        P_diag = [0, 0, 0, 0, 0, 0, 0, 0]
        N_diag = [(0.25/50)^2, (0.25/50)^2]
        W_diag = [(0.5/180*pi)^2, (0.5/180*pi)^2, (0.5/180*pi)^2]
        step_width = 0.002
    end

    properties (DiscreteState)
        x
        P
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.N = diag(obj.N_diag);
            obj.W = diag(obj.W_diag);
            
            global current_u current_A
            current_u = zeros(2, 1);
            current_A = zeros(8, 8);
        end

        function x = stepImpl(obj, y, u)
            global current_u current_A
            current_u = u;
            %f = @(t_, x_) system_f(x_, u);
            
            [~, x_ode] = ode45(@ExtendedKalmanFilter.f, [0, obj.step_width/2, obj.step_width], obj.x);
            x_predicted = x_ode(end, :)';
            A = compute_A_full(obj.x(1), obj.x(2), obj.x(3), obj.x(4), obj.x(5), obj.x(6), obj.x(7), obj.x(8), u(1), u(2));
            B = compute_B_full(obj.x(1), obj.x(2), obj.x(3), obj.x(4), obj.x(5), obj.x(6), obj.x(7), obj.x(8), u(1), u(2));
            C = [1 0 0 0 0 0 0 0; 0 1 0 0 0 0 0 0; 0 0 1 0 0 0 0 0];
            D = [0 0; 0 0; 0 0];
            
            current_A = A;
            F = expm(A * obj.step_width);
            H = integrate_matrix(@ExtendedKalmanFilter.expm_A, 0, obj.step_width, 10)*B;
            
            P_predicted = F * obj.P * F' + H * obj.N * H';
            
            y_predicted = C * x_predicted;
            
            K = P_predicted * C' * inv(C * P_predicted * C' + obj.W);
            x_corrected = x_predicted + K * (y - y_predicted);
            P_corrected = (eye(8) - K * C) * P_predicted;
            
            obj.x = x_corrected;
            obj.P = P_corrected;
            
            x = x_corrected;
        end

        function resetImpl(obj)
            obj.x = obj.x0;
            obj.P = diag(obj.P_diag);
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
    
        function [in1, in2] = getInputNamesImpl(~)
            in1 = 'Measurement';
            in2 = 'Control';
        end
        
        function [out_name] = getOutputNamesImpl(~)
            out_name = 'State Estimate';
        end
        
        function [sz, dt, cp] = getDiscreteStateSpecificationImpl(~, prop_name)
            if strcmp(prop_name, 'x')
                sz = [8, 1];
                dt = 'double';
                cp = false;
            elseif strcmp(prop_name, 'P')
                sz = [8, 8];
                dt = 'double';
                cp = false;
            end
        end
    end
    
    methods (Static)
        function out = f(t, x)
            global current_u
            
            out = system_f(x, current_u);
        end
        
        function out = expm_A(t)
            global current_A
            
            out = expm(current_A*t);
        end
    end
end
