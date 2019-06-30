function plot_meas_and_fit( sys, meas_data, title )
sim_data = sim(sys, meas_data);

figure('Name', title)
hold on
plot(meas_data.SamplingInstants, meas_data.OutputData)
plot(sim_data.SamplingInstants, sim_data.OutputData)
legend({'Measured', 'Fitted Model Output'})
grid on
end

