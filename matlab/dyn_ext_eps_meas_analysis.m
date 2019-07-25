load dyn_ext_eps_static_meas

figure
subplot(211)
hold on
grid on

active_indices = log.signals(6).values == -1;
t_start = min(log.time(active_indices));
t_meas = log.time(active_indices) - t_start;

plot(t, rad2deg(eps(:, 1)))
plot(t_meas, rad2deg(log.signals(4).values(active_indices)))

subplot(212)
hold on
grid on
plot(t_meas, debug_out.signals(1).values(1,:), 'DisplayName', 'diff eps')
plot(t_meas, debug_out.signals(1).values(2,:), 'DisplayName', 'diff deps')
plot(t_meas, debug_out.signals(1).values(5,:), 'DisplayName', 'v1')
plot(t_meas, debug_out.signals(1).values(8,:), 'DisplayName', 'ddFs')
plot(t_meas, debug_out.signals(1).values(7,:), 'DisplayName', 'dFs')

legend show