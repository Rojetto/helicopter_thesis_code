p1 = 0.3117/2;
q1 = 0.9247/2;
p2 = 0.1396/2;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

sample_decimation = 20;

%% get phi parameters for exp1 but ignore friction coefficient result

exp1_files = {'exp1_open_rect_a1_f0_3',
'exp1_open_rect_a0_5_f0_2',
'exp1_open_sine_a0_5_f0_2',
'exp1_open_sine_a0_5_f0_2_higher_vs',
'exp1_open_sine_a1_f0_2',
'exp1_feedback_sine_a25_f0_3',
'exp1_feedback_sine_a40_f0_2',
'exp1_feedback_sine_a40_f0_3'};

exp1_ps = zeros(3, 0);
exp1_reports = {};

for i=1:numel(exp1_files)
    file_name = exp1_files{i};
    [t, phi, ~, ~, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff(file_name, 0, 30);

    % define grey box model
    id_sys = idnlgrey('grey_exp1', [1, 2, 2], [0.045; 0.005; 0.05], [0; 0]);
    id_sys.Parameters(1).Minimum = 0;
    id_sys.Parameters(2).Minimum = 0;
    id_sys.Parameters(3).Minimum = 0;
    id_data = iddata([phi], [uf, ub], 0.002);
    id_data = resample(id_data, 1, sample_decimation);


    % estimate model parameters
    sys = nlgreyest(id_data, id_sys);

    exp1_ps(:, end+1) = sys.Report.Parameters.ParVector;
    exp1_reports{end+1} = sys.Report;

    plot_meas_and_fit(sys, id_data, file_name)
end

%% get friction coefficient for phi using free swing experiment

%data
t0 = 10;
phi0 = 1.0519;
dphi0 = 0;
[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_free_swing_large', t0);
phi = phi - mean(phi);
id_data = iddata([phi], [uf, ub], 0.002);
id_data = resample(id_data, 1, sample_decimation);

%system
p1_mean = mean(exp1_ps(1,:));
p2_mean = mean(exp1_ps(2,:));
p3_mean = mean(exp1_ps(3,:));
id_sys = idnlgrey('grey_exp1', [1, 2, 2], [p1_mean, p2_mean, p3_mean], [phi0; dphi0]);
id_sys.Parameters(1).Fixed = true;
id_sys.Parameters(2).Minimum = 0;
id_sys.Parameters(3).Fixed = false;

%actual estimation
sys = nlgreyest(id_data, id_sys);
exp1_reports{end+1} = sys.Report;

plot_meas_and_fit(sys, id_data, 'Phi free swing')

disp('Friction coefficient for phi:')
mu_phi = sys.Report.Parameters.ParVector(2)

%% get eps parameters for exp3 but ignore friction coefficient result

exp3_files = {'exp3_feedback_sine_a10_f0_1',
'exp3_feedback_sine_a10_f0_2',
'exp3_open_rect_a1_f0_2_off2',
'exp3_open_rect_a3_f0_2',
'exp3_open_rect_a5_f0_4',
'exp3_open_sine_a1_f0_2_off2',
'exp3_open_sine_a3_f0_2',
'exp3_open_sine_a6_f0_4'};


exp3_ps = zeros(3, 0);
exp3_reports = {};

for i=1:numel(exp3_files)
    file_name = exp3_files{i};
    [t, ~, ~, ~, eps, ~, ~, ~, ~, ~, uf, ub] = load_and_diff(file_name, 0, 30);

    % define grey box model
    id_sys = idnlgrey('grey_exp3', [1, 2, 2], [0.34; 0.01; 0.38], [0; 0]);
    id_sys.Parameters(1).Minimum = 0;
    id_sys.Parameters(2).Minimum = 0;
    id_sys.Parameters(3).Minimum = 0;
    id_data = iddata([eps], [uf, ub], 0.002);
    id_data = resample(id_data, 1, sample_decimation);


    % estimate model parameters
    sys = nlgreyest(id_data, id_sys);

    exp3_ps(:, end+1) = sys.Report.Parameters.ParVector;
    exp3_reports{end+1} = sys.Report;

    plot_meas_and_fit(sys, id_data, file_name)
end


%% get friction coefficient for eps using free swing experiment

%data
t0 = 9;
eps0 = 0.4542;
deps0 = 0;
[t, ~, ~, ~, eps, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp3_free_swing_large', t0);
eps = eps - mean(eps);
id_data = iddata([eps], [uf, ub], 0.002);
id_data = resample(id_data, 1, sample_decimation);

%system
p1_mean = mean(exp3_ps(1,:));
p2_mean = mean(exp3_ps(2,:));
p3_mean = mean(exp3_ps(3,:));
id_sys = idnlgrey('grey_exp3', [1, 2, 2], [p1_mean, p2_mean, p3_mean], [eps0; deps0]);
id_sys.Parameters(1).Fixed = true;
id_sys.Parameters(2).Minimum = 0;
id_sys.Parameters(3).Fixed = false;

%actual estimation
sys = nlgreyest(id_data, id_sys);
exp3_reports{end+1} = sys.Report;

plot_meas_and_fit(sys, id_data, 'eps free swing')

disp('Friction coefficient for eps:')
mu_eps = sys.Report.Parameters.ParVector(2)


%% get exp0 only eps parameters
t0 = 10.73;
te = 40;
eps0 = 0.018;
deps0 = 0;
[t, ~, ~, ~, eps, deps, ~, ~, ~, ~, uf, ub] = load_and_diff('exp0_sine_elevation', t0, te);
id_data = iddata([eps], [uf, ub], 0.002);
id_data = resample(id_data, 1, sample_decimation);

id_sys = idnlgrey('grey_exp0_only_eps', [1, 2, 2], [1, mu_eps, 1, 1], [eps0; deps0]);
id_sys.Parameters(1).Minimum = 0;
id_sys.Parameters(2).Fixed = true;
id_sys.Parameters(3).Minimum = 0;
id_sys.Parameters(4).Minimum = 0;

sys = nlgreyest(id_data, id_sys);

plot_meas_and_fit(sys, id_data, 'eps sine')

exp0_only_eps_report = sys.Report;

p_eps_1 = sys.Report.Parameters.ParVector(1)
p_eps_2 = sys.Report.Parameters.ParVector(3)
p_eps_3 = sys.Report.Parameters.ParVector(4)


%% get exp0 phi and lambda parameters
t0 = 10;
te = 30;
[t, phi, dphi, ~, eps, deps, ~, lamb, dlamb, ~, uf, ub] = load_and_diff('exp0_sine_travel', t0, te);
id_data = iddata([phi, eps, lamb], [uf, ub], 0.002);
id_data = resample(id_data, 1, sample_decimation);

init_params = [1, mu_phi, 1, p_eps_1, mu_eps, p_eps_2, p_eps_3, 1, 1];
init_states = [phi(1); eps(1); lamb(1); dphi(1); deps(1); dlamb(1)];

id_sys = idnlgrey('grey_exp0', [3, 2, 6], init_params, init_states);
id_sys.Parameters(1).Minimum = 0;
id_sys.Parameters(2).Fixed = true;
id_sys.Parameters(3).Minimum = 0;
id_sys.Parameters(4).Fixed = true;
id_sys.Parameters(5).Fixed = true;
id_sys.Parameters(6).Fixed = true;
id_sys.Parameters(7).Fixed = true;
id_sys.Parameters(8).Minimum = 0;
id_sys.Parameters(9).Minimum = 0;

opt = nlgreyestOptions;
opt.SearchOption.MaxIter = 100;
sys = nlgreyest(id_data, id_sys, opt);

sim_data = sim(sys, id_data);

figure
hold on
plot(id_data.SamplingInstants, id_data.OutputData)
plot(sim_data.SamplingInstants, sim_data.OutputData)
legend({'meas phi', 'meas eps', 'meas lamb', 'fit phi', 'fit eps', 'fit lamb'})

exp0_phi_lamb_report = sys.Report;


%%
disp('Done with everything')

