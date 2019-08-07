active_indices = log.signals(6).values == -1;
t_start = min(log.time(active_indices));
t_meas = log.time(active_indices) - t_start;

eps_traj = interp1(t, eps(:,1), t_meas);

phi_est = log.signals(7).values(active_indices);
eps_est = log.signals(8).values(active_indices);
lamb_est = log.signals(9).values(active_indices);
dphi_est = log.signals(10).values(active_indices);
deps_est = log.signals(11).values(active_indices);
dlamb_est = log.signals(12).values(active_indices);

%%
Fs = debug_out.signals(1).values(6,:)';
dFs = debug_out.signals(1).values(7,:)';

x = [phi_est, eps_est, lamb_est, dphi_est, deps_est, dlamb_est, Fs, dFs];
z = zeros(size(x));

c_buildTapes_mex();

for i = 1:size(x, 1)
    z(i, :) = c_calcPhi_mex(x(i, :)');
end

%%
z = debug_out.signals(1).values(3:10,:)';
sigma1 = debug_out.signals(1).values(11,:)';
sigma2 = debug_out.signals(1).values(12,:)';

%%
figure

subplot(311)
hold on
grid on
plot(t_meas, rad2deg(eps_est))
plot(t_meas, rad2deg(eps_traj))

subplot(312)
hold on
grid on
plot(t_meas, z(:,1:4))

subplot(313)
hold on
grid on
plot(t_meas, sigma1)


%%
figure

subplot(311)
hold on
grid on
plot(t_meas, eps_est)
plot(t_meas, eps_traj)

subplot(312)
hold on
grid on
plot(t_meas, Fs)


subplot(313)
hold on
grid on
plot(t_meas, debug_out.signals(1).values(5,:))

%%
figure

subplot(411)
hold on
grid on
plot(t_meas, eps_est)
plot(t_meas, z(:, 1))


subplot(412)
hold on
grid on
plot(t_meas, deps_est)
plot(t_meas, z(:, 2))
deps_int = cumtrapz(t_meas, z(:, 3));
plot(t_meas, deps_int)


subplot(413)
hold on
grid on
plot(t_meas, z(:, 3))
ddeps_int = cumtrapz(t_meas, z(:,4))+0.05;
plot(t_meas, ddeps_int)


subplot(414)
hold on
grid on
plot(t_meas, z(:, 4))


% subplot(212)
% hold on
% grid on
% plot(t_meas, debug_out.signals(1).values(1,:), 'DisplayName', 'diff eps')
% plot(t_meas, debug_out.signals(1).values(2,:), 'DisplayName', 'diff deps')
% plot(t_meas, debug_out.signals(1).values(5,:), 'DisplayName', 'v1')
% plot(t_meas, debug_out.signals(1).values(8,:), 'DisplayName', 'ddFs')
% plot(t_meas, debug_out.signals(1).values(7,:), 'DisplayName', 'dFs')
% 
% legend show