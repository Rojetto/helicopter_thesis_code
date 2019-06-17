function set_ramping_target_parameters( current_eps, current_lamb )
%function_block = gcb;
%ramping_controller = get_param(get_param(function_block, 'Parent'), 'Parent');
print('params set');
ramping_controller = 'heli_master_2015b/Feedback Controller/Ramping Controller';
param_values = get_param(ramping_controller, 'MaskValues');
param_values{1} = num2str(current_eps);
param_values{2} = num2str(current_lamb);
set_param(ramping_controller, 'MaskValues', param_values);
end

