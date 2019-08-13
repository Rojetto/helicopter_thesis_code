debug_out_3d = debug_out.signals(1).values;
debug_out_2d = reshape(debug_out_3d, size(debug_out_3d, 1), size(debug_out_3d, 3))';
debug_out_t = debug_out.time;

c_buildTapes_mex();

%%
n_ts = length(debug_out_t);
x_alt = debug_out_2d(:, 1:8);
x_est = debug_out_2d(:, 9:16);
z_est = debug_out_2d(:, 17:24);

z_trafo = zeros(size(z_est));
for i=1:size(z_trafo, 1)
    z_trafo(i, :) = c_calcPhi_mex(x_alt(i, :)');
end

phi_jac_cond = zeros(n_ts, 1);
for i=1:n_ts
    phi_jac_cond(i) = cond(c_calcPhiJacobian_mex(x_est(i, :)'));
end

%%
figure
plot(debug_out_t, phi_jac_cond);

%%

figure

subplot(421)
hold on
plot(debug_out_t, rad2deg(z_est(:, 1)))
plot(debug_out_t, rad2deg(z_trafo(:, 1)))
plot(debug_out_t, rad2deg(x_alt(:, 2)))
grid
legend({'z est', 'z trafo', 'meas'})

subplot(423)
hold on
plot(debug_out_t, rad2deg(z_est(:, 2)))
plot(debug_out_t, rad2deg(z_trafo(:, 2)))
plot(debug_out_t, rad2deg(x_alt(:, 5)))
grid
legend({'z est', 'z trafo', 'meas'})

subplot(425)
hold on
plot(debug_out_t, rad2deg(z_est(:, 3)))
plot(debug_out_t, rad2deg(z_trafo(:, 3)))
grid

subplot(427)
hold on
plot(debug_out_t, rad2deg(z_est(:, 4)))
plot(debug_out_t, rad2deg(z_trafo(:, 4)))
grid

subplot(422)
hold on
plot(debug_out_t, rad2deg(z_est(:, 5)))
plot(debug_out_t, rad2deg(z_trafo(:, 5)))
plot(debug_out_t, rad2deg(x_alt(:, 3)))
grid
legend({'z est', 'z trafo', 'meas'})

subplot(424)
hold on
plot(debug_out_t, rad2deg(z_est(:, 6)))
plot(debug_out_t, rad2deg(z_trafo(:, 6)))
plot(debug_out_t, rad2deg(x_alt(:, 6)))
grid
legend({'z est', 'z trafo', 'meas'})

subplot(426)
hold on
plot(debug_out_t, rad2deg(z_est(:, 7)))
plot(debug_out_t, rad2deg(z_trafo(:, 7)))
grid

subplot(428)
hold on
plot(debug_out_t, rad2deg(z_est(:, 8)))
plot(debug_out_t, rad2deg(z_trafo(:, 8)))
grid