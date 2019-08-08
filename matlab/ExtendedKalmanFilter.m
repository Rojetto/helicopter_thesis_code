classdef ExtendedKalmanFilter < matlab.System ...
        & matlab.system.mixin.Propagates
    properties (Access = private)
        N
        W
    end
    
    properties
        x0 = [0; deg2rad(-29); 0; 0; 0; 0; deg2rad(6)]
        P_diag = [1, 1, 1, 1, 1, 1, 1]
        N_diag = [ 1000, 1000 ]
        W_diag = [ 0.01, 0.0001, 0.0001 ]
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
        end

        function x = stepImpl(obj, y, u)
            ExtendedKalmanFilter.ode_f(0, zeros(7, 1), u);
            %f = @(t_, x_) system_f(x_, u);
            
            [~, x_ode] = ode45(@ExtendedKalmanFilter.ode_f, [0, obj.step_width/2, obj.step_width], obj.x);
            x_predicted = x_ode(end, :)';
            A_without_offset = compute_A_6_states(obj.x(1), obj.x(2), obj.x(3), obj.x(4), obj.x(5), obj.x(6), u(1), u(2));
            A = zeros(7);
            A(1:6,1:6) = A_without_offset;
            B_without_offset = compute_B_6_states(obj.x(1), obj.x(2), obj.x(3), obj.x(4), obj.x(5), obj.x(6), u(1), u(2));
            B = zeros(7, 2);
            B(1:6,:) = B_without_offset;
            % Output: (phi + phi_off, eps, lamb)
            C = [1 0 0 0 0 0 1; 0 1 0 0 0 0 0; 0 0 1 0 0 0 0];
            
            ExtendedKalmanFilter.expm_A(0, A);
            F = expm(A * obj.step_width);
            H = integrate_matrix(@ExtendedKalmanFilter.expm_A, 0, obj.step_width, 10)*B;
            
            P_predicted = F * obj.P * F' + H * obj.N * H';
            
            y_predicted = C * x_predicted;
            
            K = (P_predicted * C') / (C * P_predicted * C' + obj.W);
            x_corrected = x_predicted + K * (y - y_predicted);
            P_corrected = (eye(7) - K * C) * P_predicted;
            
            % clamp phi_off for weird edge case where it completely
            % diverges
            phi_off_limit = 10 / 180 * pi;
            x_corrected(7) = max(min(x_corrected(7), phi_off_limit), -phi_off_limit);
            
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
            out1 = 7;
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
                sz = [7, 1];
                dt = 'double';
                cp = false;
            elseif strcmp(prop_name, 'P')
                sz = [7, 7];
                dt = 'double';
                cp = false;
            end
        end
    end
    
    methods (Static)
        function out = ode_f(t, x, u)
            persistent current_u
            
            if isempty(current_u)
                current_u = zeros(2, 1);
            end
            
            if nargin == 3
                current_u = u;
            end
            
            out = zeros(7, 1);
            dx = system_f([x(1:6); 0; 0], current_u);
            out(1:6) = dx(1:6);
            out(7) = 0; % dphi_off = 0
        end
        
        function out = expm_A(t, A)
            persistent current_A
            
            if isempty(current_A)
                current_A = zeros(7, 7);
            end
            
            if nargin == 2
                current_A = A;
            end
            
            out = expm(current_A*t);
        end
    end
end
