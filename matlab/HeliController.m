classdef (Abstract) HeliController < matlab.System ...
        & matlab.system.mixin.Propagates
    
    properties (Access=private)
        is_initialized = false
    end
    
    properties (Access=protected)
        trajectory
    end
    
    methods (Abstract)
        initialize(obj)
        
        control(obj, t, x)
    end
    
    methods
        function setAllParameters(obj, param_values)
            for i= 1:2:numel(param_values)
                name = param_values{i};
                value = param_values{i+1};
                
                obj.(name) = value;
            end
        end
        
        function out = getParametersAndValues(obj)
            propGroups = obj.getPropertyGroups();
            
            out = cell(1, propGroups.NumProperties*2);
            
            for i = 1:propGroups.NumProperties
                prop_name = propGroups.PropertyList{i};
                prop_value = obj.(prop_name);
                
                out{i*2 - 1} = prop_name;
                out{i*2} = prop_value;
            end
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, t, x, t_d, phi_d, eps_d, lamb_d, vf_d, vb_d)
            obj.trajectory = struct('t', t_d, 'phi', phi_d, 'eps', eps_d, 'lamb', lamb_d, 'vf', vf_d, 'vb', vb_d);
        end
        
        function [u, debug_out] = stepImpl(obj, t, x, t_d, phi_d, eps_d, lamb_d, vf_d, vb_d)
            if ~obj.is_initialized
                obj.trajectory.t = t_d;
                obj.trajectory.phi = phi_d;
                obj.trajectory.eps = eps_d;
                obj.trajectory.lamb = lamb_d;
                obj.trajectory.vf = vf_d;
                obj.trajectory.vb = vb_d;
                
                obj.initialize()
                obj.is_initialized = true;
            end
            
            [u, debug_out_controller] = obj.control(t, x);
            
            if length(debug_out_controller) > 0
                debug_out = debug_out_controller;
            else
                debug_out = 0;
            end
        end
        
        function [out1, out2] = isOutputFixedSizeImpl(~)
            out1 = true;
            out2 = true;
        end
        
        function [out1, out2] = isOutputComplexImpl(~)
            out1 = false;
            out2 = false;
        end
        
        function [out1, out2] = getOutputSizeImpl(obj)
            out1 = 2;
            out2 = obj.getDebugOutputSize();
        end
        
        function [out1, out2] = getOutputDataTypeImpl(~)
            out1 = 'double';
            out2 = 'double';
        end
    
        function [in1, in2, in3, in4, in5, in6, in7, in8] = getInputNamesImpl(~)
            in1 = 'Time';
            in2 = 'State';
            in3 = 'Traj Time';
            in4 = 'Traj Phi';
            in5 = 'Traj Eps';
            in6 = 'Traj Lamb';
            in7 = 'Traj Vf';
            in8 = 'Traj Vb';
        end
        
        function [out1, out2] = getOutputNamesImpl(~)
            out1 = 'Control';
            out2 = 'Debug';
        end
        
        % override in subclass
        function out = getDebugOutputSize(~)
            out = 1;
        end
    end
end

