function plot_meas_and_fit( sys, meas_data, title )
n_experiments = numel(meas_data.ExperimentNames);

sim_data = sim(sys, meas_data);

for i=1:n_experiments
    if exist('title', 'var')
        fig_title = [title ', ' meas_data.ExperimentNames{i}];
    else
        fig_title = meas_data.ExperimentNames{i};
    end
    
    figure('Name', fig_title)
    hold on

    meas_t = get_if_cell(meas_data.SamplingInstants, i);
    meas_y = get_if_cell(meas_data.OutputData, i);
    sim_t = get_if_cell(sim_data.SamplingInstants, i);
    sim_y = get_if_cell(sim_data.OutputData, i);

    plot(meas_t, meas_y)
    plot(sim_t, sim_y)
    n_outputs = numel(meas_data.OutputName);
    legend_names = cell(n_outputs, 1);
    for j=1:n_outputs
        legend_names{j} = ['Meas: ' meas_data.OutputName{j}];
        legend_names{j+n_outputs} = ['Sim: ' sim_data.OutputName{j}];
    end
    legend(legend_names)

    params = sys.Parameters;
    params = [params.Value];

    xl = xlim();
    yl = ylim();

    text(xl(1)+0.1*(xl(2)-xl(1)), yl(1)+0.1*(yl(2)-yl(1)), num2str(params));

    grid on
end

function val = get_if_cell(cell_obj, index)
    if iscell(cell_obj)
        val = cell_obj{index};
    else
        val = cell_obj;
    end
end
end

