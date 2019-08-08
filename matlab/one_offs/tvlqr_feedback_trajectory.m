n = 6;
m = 2;

K = zeros(length(t), m, n);

for i=1:length(t)
    t_i = t(i);
    
    [A_i, B_i] = TimeVariantLQR.get_SS_at_time(trajectory, t_i);
    tau_i = tau_e - t_i;
    P_triu_i = interp1(riccati_tau, riccati_P_triu_tau, tau_i);
    P_i = TimeVariantLQR.triu_to_full(P_triu_i);
    K_i = R_inv * B_i' * P_i;
    
    K(i, :, :) = K_i;
end

figure
subplot(311)
hold on
grid on
plot(t, rad2deg(phi(:, 1)))
plot(t, rad2deg(eps(:, 1)))
plot(t, rad2deg(lamb(:, 1)))

subplot(312)
hold on
grid on
plot(t, reshape(K(:, :, 1:3), length(t), 6))
legend('uf phi', 'ub phi', 'uf eps', 'ub eps', 'uf lamb', 'ub lamb')

subplot(313)
hold on
grid on
plot(t, reshape(K(:, :, 4:6), length(t), 6))
legend('uf dphi', 'ub dphi', 'uf deps', 'ub deps', 'uf dlamb', 'ub dlamb')