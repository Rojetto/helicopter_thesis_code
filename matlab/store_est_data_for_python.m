function store_est_data_for_python(sys, meas_data, file_name)
n_experiments = numel(meas_data.ExperimentNames);

sim_data = sim(sys, meas_data);

params = sys.Parameters;
params = [params.Value];

clips = struct('meas_t', {}, 'meas_u', {}, 'meas_y', {}, 'sim_y', {});

for i=1:n_experiments
    meas_t = get_if_cell(meas_data.SamplingInstants, i);
    meas_u = get_if_cell(meas_data.InputData, i);
    meas_y = get_if_cell(meas_data.OutputData, i);
    sim_y = get_if_cell(sim_data.OutputData, i);
    
    clips(end+1) = struct('meas_t', meas_t, 'meas_u', meas_u,...
                          'meas_y', meas_y, 'sim_y', sim_y);
end

save(file_name, 'params', 'clips');

function val = get_if_cell(cell_obj, index)
    if iscell(cell_obj)
        val = cell_obj{index};
    else
        val = cell_obj;
    end
end
end

