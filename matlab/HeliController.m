classdef (Abstract) HeliController < matlab.System ...
        & matlab.system.mixin.SampleTime ...
        & matlab.system.mixin.Propagates    
    methods (Abstract)
        initialize(obj)
        
        control(obj, t, x, elevation_traj, lambda_traj)
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
        function setupImpl(obj)
            obj.initialize()
        end
        
        function u = stepImpl(obj, x, elevation_traj, lambda_traj)
            t = obj.getCurrentTime();
            u = obj.control(t, x, elevation_traj, lambda_traj);
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
    
        function [x_name, etraj_name, ltraj_name] = getInputNamesImpl(~)
            x_name = 'State';
            etraj_name = 'Elevation';
            ltraj_name = 'Travel';
        end
        
        function [out_name] = getOutputNamesImpl(~)
            out_name = 'Control';
        end
    end
end

