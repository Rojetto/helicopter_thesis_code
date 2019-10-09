%% configuration
Q = diag([2, 100, 150, 1, 0.2, 20]);
R = diag([1, 1]);
S = zeros(6);

riccati_step_width = 0.1;

auto_compute_S = true;


%% preparation
R_inv = inv(R);
trajectory = struct('t', t, ...
                    'phi', phi, ...
                    'eps', eps, ...
                    'lamb', lamb, ...
                    'vf', vf, ...
                    'vb', vb);
tau_e = t(end);
n_taus = floor(tau_e/riccati_step_width + 1);
riccati_tau = linspace(0, tau_e, n_taus);

P_triu_0 = TimeVariantLQR.full_to_triu(S);

%% solving the riccati equation

[~, riccati_P_triu_tau] = ode45(@(tau, P_triu) TimeVariantLQR.riccati_rhs(tau, P_triu, trajectory, R_inv, Q), riccati_tau, P_triu_0);
K_traj = zeros(n_taus, 2, 6);

if auto_compute_S
    P_triu_0 = riccati_P_triu_tau(end, :);
    [~, riccati_P_triu_tau] = ode45(@(tau, P_triu) TimeVariantLQR.riccati_rhs(tau, P_triu, trajectory, R_inv, Q), riccati_tau, P_triu_0);
end

%% K trajectory for plots
for i=1:n_taus
    ti = riccati_tau(i);
    [~, Bi] = TimeVariantLQR.get_SS_at_time(trajectory, ti);
    P_triu_i = riccati_P_triu_tau(n_taus - i + 1, :);
    Pi = TimeVariantLQR.triu_to_full(P_triu_i);
    Ki = R_inv * Bi' * Pi;
    K_traj(i, :, :) = Ki;
end