active_indices = log.signals(6).values == -1;
t_start = min(log.time(active_indices));
t_meas = log.time(active_indices) - t_start;

%plot(t, rad2deg(eps(:, 1)))
%plot(t_meas, rad2deg(log.signals(4).values(active_indices)))

phi_traj = interp1(t, phi(:, 1), t_meas);
eps_traj = interp1(t, eps(:, 1), t_meas);
lamb_traj = interp1(t, lamb(:, 1), t_meas);

figure

subplot(231)
hold on
plot(t_meas, rad2deg(phi_traj))
plot(t_meas, rad2deg(log.signals(7).values(active_indices)))
grid on

subplot(234)
plot(t_meas, rad2deg(log.signals(7).values(active_indices) - phi_traj))
grid on

subplot(232)
hold on
plot(t_meas, rad2deg(eps_traj))
plot(t_meas, rad2deg(log.signals(8).values(active_indices)))
grid on

subplot(235)
plot(t_meas, rad2deg(log.signals(8).values(active_indices) - eps_traj))
grid on

subplot(233)
hold on
plot(t_meas, rad2deg(lamb_traj(:, 1)))
plot(t_meas, rad2deg(log.signals(9).values(active_indices)))
grid on

subplot(236)
plot(t_meas, rad2deg(log.signals(9).values(active_indices) - lamb_traj))
grid on

figure
subplot(211)
hold on
grid on
plot(t_meas, rad2deg(eps_traj(:, 1)))
plot(t_meas, rad2deg(log.signals(8).values(active_indices)))
plot(t_meas, rad2deg(lamb_traj(:, 1)))
plot(t_meas, rad2deg(log.signals(9).values(active_indices)))

subplot(212)
hold on
grid on
vf_meas = log.signals(1).values(:);
vb_meas = log.signals(2).values(:);
plot(t_meas, vf_meas(active_indices))
plot(t_meas, vb_meas(active_indices))

figure
subplot(311)
plot(t_meas, rad2deg(log.signals(10).values(active_indices)))
grid on

subplot(312)
plot(t_meas, rad2deg(log.signals(11).values(active_indices)))
grid on

subplot(313)
plot(t_meas, rad2deg(log.signals(12).values(active_indices)))
grid on