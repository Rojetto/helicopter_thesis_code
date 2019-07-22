%% configuration
Q = diag([3, 50, 20, 2, 1, 5, 1, 1, 0.1, 10, 10]);
R = diag([1, 1]);
S = diag([3, 50, 20, 2, 1, 5, 1, 1, 0.1, 10, 10]);

riccati_step_width = 0.1;


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
P_triu_0 = TimeVariantLQRI.full_to_triu(S);

%% solving the riccati equation
[~, riccati_P_triu_tau] = ode45(@(tau, P_triu) TimeVariantLQRI.riccati_rhs(tau, P_triu, trajectory, R_inv, Q), riccati_tau, P_triu_0);