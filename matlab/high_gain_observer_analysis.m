load high_gain_with_difference.mat

t = z_est.time;
z_est = z_est.signals(1).values(:, :)';
x_sim = x_sim.signals(1).values(:, :)';

z_int = cumtrapz(t, z_est(:, 2:4));

z_trans = zeros(size(x_sim));

c_buildTapes_mex();

for i=1:size(x_sim, 1)
    z_trans(i, :) = c_calcPhi_mex(x_sim(i, :)');
end

figure

subplot(411)
hold on
grid on
plot(t, z_est(:, 1))
plot(t, z_trans(:, 1))
plot(t, z_int(:, 1) + z_est(end, 1) - z_int(end, 1))

subplot(412)
hold on
grid on
plot(t, z_est(:, 2))
plot(t, z_trans(:, 2))
plot(t, z_int(:, 2) + z_est(end, 2) - z_int(end, 2))

subplot(413)
hold on
grid on
plot(t, z_est(:, 3))
plot(t, z_trans(:, 3))
plot(t, z_int(:, 3) + z_est(end, 3) - z_int(end, 3))

subplot(414)
hold on
grid on
plot(t, z_est(:, 4))
plot(t, z_trans(:, 4))