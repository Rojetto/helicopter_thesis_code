if ~exist('rotor_force_data', 'var')
    rotor_force_data.v = [];
    rotor_force_data.eps = [];
end

rotor_force_data.v(end+1) = 0.5* (logsout.getElement('vf').Values.Data(end) + logsout.getElement('vb').Values.Data(end));
rotor_force_data.eps(end+1) = logsout.getElement('meas_eps').Values.Data(end);