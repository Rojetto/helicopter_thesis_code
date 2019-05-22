classdef (Abstract) HeliController < matlab.System ...
        & matlab.system.mixin.SampleTime ...
        & matlab.system.mixin.Propagates
    
    methods (Abstract)
        initialize(obj, t_d, phi_d, eps_d, lamb_d, vf_d, vb_d)
        
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
        function setupImpl(obj, x, t_d, phi_d, eps_d, lamb_d, vf_d, vb_d) %#ok<*INUSL>
            obj.initialize(t_d, phi_d, eps_d, lamb_d, vf_d, vb_d)
        end
        
        function u = stepImpl(obj, x, t_d, phi_d, eps_d, lamb_d, vf_d, vb_d) %#ok<INUSD>
            t = obj.getCurrentTime();
            u = obj.control(t, x);
        end
        
        function [out1] = isOutputFixedSizeImpl(~)
            out1 = true;
        end
        
        function [out1] = isOutputComplexImpl(~)
            out1 = false;
        end
        
        function [out1] = getOutputSizeImpl(~)
            out1 = 2;
        end
        
        function [out1] = getOutputDataTypeImpl(~)
            out1 = 'double';
        end
    
        function [in1, in2, in3, in4, in5, in6, in7] = getInputNamesImpl(~)
            in1 = 'State';
            in2 = 'Traj Time';
            in3 = 'Traj Phi';
            in4 = 'Traj Eps';
            in5 = 'Traj Lamb';
            in6 = 'Traj Vf';
            in7 = 'Traj Vb';
        end
        
        function [out_name] = getOutputNamesImpl(~)
            out_name = 'Control';
        end
    end
end

