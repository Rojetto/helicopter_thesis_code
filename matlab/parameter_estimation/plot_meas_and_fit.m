function plot_meas_and_fit( sys, meas_data, title )
n_experiments = numel(meas_data.ExperimentNames);

sim_data = sim(sys, meas_data);

for i=1:n_experiments
figure('Name', [title ', ' meas_data.ExperimentNames{i}])
hold on
plot(meas_data.SamplingInstants{i}, meas_data.OutputData{i})
plot(sim_data.SamplingInstants{i}, sim_data.OutputData{i})
legend({'Measured', 'Fitted Model Output'})

params = sys.Parameters;
params = [params.Value];

xl = xlim();
yl = ylim();

text(xl(1)+0.1*(xl(2)-xl(1)), yl(1)+0.1*(yl(2)-yl(1)), num2str(params));

grid on
end
end

