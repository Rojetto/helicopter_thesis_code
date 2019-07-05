p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

g = 9.81;

sample_decimation = 20;

%% get phi parameters for exp1 but ignore friction coefficient result

files = {'exp1_open_rect_a1_f0_3',
'exp1_open_rect_a0_5_f0_2',
'exp1_open_sine_a0_5_f0_2',
'exp1_open_sine_a0_5_f0_2_higher_vs',
'exp1_open_sine_a1_f0_2',
'exp1_feedback_sine_a25_f0_3',
'exp1_feedback_sine_a40_f0_2',
'exp1_feedback_sine_a40_f0_3'};

[clips, data, inits] = make_experiment_clips(files, 'phi');

sys_init = idnlgrey('grey_exp1', [1, 2, 2], [0.044; 0.0022; 0.033], inits);

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp1 Parameters except friction')

% resulting parameters
exp1_p_phi_1 = sys_est.Report.Parameters.ParVector(1);
exp1_p_phi_2 = sys_est.Report.Parameters.ParVector(2);

%% get friction coefficient for phi using free swing experiment

[clips, data, inits] = make_experiment_clips(...
    {'exp1_free_swing_large', 'exp1_free_swing_small'}, 'phi', ...
    'StartTimes', [10, 5], 'EndTimes', [41, 26]);

sys_init = idnlgrey('grey_exp1', [1, 2, 2], [exp1_p_phi_1; exp1_p_phi_2; 0.04], inits);
sys_init.Parameters(1).Fixed = true;
sys_init.Parameters(2).Fixed = false;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp1 mu_phi')

% resulting parameters
mu_phi = sys_est.Report.Parameters.ParVector(3);

%% get eps parameters for exp3 but ignore friction coefficient result

files = {'exp3_feedback_sine_a10_f0_1',
'exp3_feedback_sine_a10_f0_2',
'exp3_open_rect_a1_f0_2_off2',
'exp3_open_rect_a3_f0_2',
'exp3_open_rect_a5_f0_4',
'exp3_open_sine_a1_f0_2_off2',
'exp3_open_sine_a3_f0_2',
'exp3_open_sine_a6_f0_4'};

[clips, data, inits] = make_experiment_clips(files, 'eps');

sys_init = idnlgrey('grey_exp3', [1, 2, 2], [1.3; 1.5; 0.05], inits);

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp3 Parameters except friction')

% resulting parameters
exp3_p_eps_1 = sys_est.Report.Parameters.ParVector(1);
exp3_p_eps_2 = sys_est.Report.Parameters.ParVector(2);


%% get friction coefficient for eps using free swing experiment

[clips, data, inits] = make_experiment_clips(...
    {'exp3_free_swing_large', 'exp3_free_swing_small'}, 'eps', ...
    'StartTimes', [9, 6.5], 'EndTimes', [100, 70]);

sys_init = idnlgrey('grey_exp3', [1, 2, 2], [exp3_p_eps_1; exp3_p_eps_2; 0.076], inits);
sys_init.Parameters(1).Fixed = true;
sys_init.Parameters(2).Fixed = false;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp3 mu_eps')

% resulting parameters
mu_eps = sys_est.Report.Parameters.ParVector(3);


%% get exp5 p_eps_1
files = {'exp5_feedback_rect_a5_f0_1',
'exp5_feedback_rect_a10_f0_1',
'exp5_feedback_sine_a10_f0_2',
'exp5_feedback_sine_a10_f0_4',
'exp5_open_rect_a0_8_f0_05_off7',
'exp5_open_rect_a1_f0_05_off6',
'exp5_open_sine_a0_1_f0_2_off7',
'exp5_open_sine_a0_5_f0_2_off7'};

start_times = [4, 4, 4, 4, 12, 4, 15, 7];

[clips, data, inits] = make_experiment_clips(files, 'eps', ...
    'StartTimes', start_times, 'EndTimes', end_times, 'ResamplingDecimation', 20);

sys_init = idnlgrey('grey_exp5', [1, 2, 2], [0.838; 1.33; 0.838; mu_eps], inits);
sys_init.Parameters(4).Fixed = true;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp5 p_eps_1')

% resulting parameters
p_eps_1 = sys_est.Report.Parameters.ParVector(1);

%% get p_eps_2 and p_eps_3
[clips, data, inits] = make_experiment_clips({'elevation_steps'}, 'eps', ...
    'StartTimes', 5, 'EndTimes', 171, 'ResamplingDecimation', 4);

sys_init = idnlgrey('grey_exp5', [1, 2, 2], [p_eps_1; 1.33; 0.838; mu_eps], inits);
sys_init.Parameters(1).Fixed = true;
sys_init.Parameters(4).Fixed = true;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp5 p_eps_2 and p_eps_3')

% resulting parameters
p_eps_2 = sys_est.Report.Parameters.ParVector(2);
p_eps_3 = sys_est.Report.Parameters.ParVector(3);

%% get exp4 phi parameters
exp4_phi_files = {'exp4_phi_feedback_rect_a5_f0_2_Vs4',
'exp4_phi_feedback_rect_a20_f0_2_Vs4',
'exp4_phi_feedback_rect_a20_f0_2_Vs6',
'exp4_phi_feedback_sine_a20_f0_2_Vs6',
'exp4_phi_feedback_sine_a45_f0_2_Vs6',
'exp4_phi_feedback_sine_a45_f0_4_Vs6'};

t0s = [0, 0, 0, 0, 0, 0];

exp4_phi_ps = zeros(4, 0);
exp4_phi_reports = {};

for i=1:numel(exp4_phi_files)
    file_name = exp4_phi_files{i};
    [t, phi, dphi, ~, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff(file_name, t0s(i), 5);

    init_params = [0.15, -0.007, mu_phi, 0];
    init_states = [phi(1); dphi(1)];
    
    % define grey box model
    id_sys = idnlgrey('grey_exp4_only_phi', [1, 2, 2], init_params, init_states);
    id_sys.Parameters(1).Minimum = 0;
    %id_sys.Parameters(2).Maximum = 0;
    id_sys.Parameters(3).Fixed = true;
    id_data = iddata([phi], [uf, ub], 0.002);
    id_data = resample(id_data, 1, sample_decimation);


    % estimate model parameters
    sys = nlgreyest(id_data, id_sys);

    exp4_phi_ps(:, end+1) = sys.Report.Parameters.ParVector;
    exp4_phi_reports{end+1} = sys.Report;

    plot_meas_and_fit(sys, id_data, file_name)
end


%% exp4 lambda parameters
exp4_lamb_files = {'exp4_lamb_feedback_rect_a10_f0_05',
'exp4_lamb_feedback_sine_a20_f0_1',
'exp4_lamb_feedback_sine_a20_f0_2_Vs7',
'exp4_lamb_open_step_phi_neg',
'exp4_lamb_open_step_phi_pos'};

t0s = [0, 0, 0, 7, 7];

exp4_lamb_ps = zeros(3, 0);
exp4_lamb_reports = {};

for i=1:numel(exp4_lamb_files)
    file_name = exp4_lamb_files{i};
    [t, phi, ~, ~, ~, ~, ~, lamb, dlamb, ~, uf, ub] = load_and_diff(file_name, t0s(i));

    init_params = [1.3, 0.045, 0];
    init_states = [lamb(1); dlamb(1)];
    
    % define grey box model
    id_sys = idnlgrey('grey_exp4_only_lamb', [1, 3, 2], init_params, init_states);
    id_sys.Parameters(1).Minimum = 1.0;
    id_sys.Parameters(1).Maximum = 1.5;
    id_sys.Parameters(2).Minimum = 0.03;
    id_sys.Parameters(2).Maximum = 0.07;
    id_sys.Parameters(3).Minimum = deg2rad(-10);
    id_sys.Parameters(3).Maximum = deg2rad(10);
    id_data = iddata([lamb], [uf, ub, phi], 0.002);
    id_data = resample(id_data, 1, sample_decimation);


    % estimate model parameters
    sys = nlgreyest(id_data, id_sys, 'Display', 'on');

    exp4_lamb_ps(:, end+1) = sys.Report.Parameters.ParVector;
    exp4_lamb_reports{end+1} = sys.Report;

    plot_meas_and_fit(sys, id_data, file_name)
end

%% calculate physical model parameters
lp
lh
mh = p_phi_1 / lp^2
lc = (p_lamb_1 - (lh^2+lp^2)*mh)/(lh*mh-p_eps_3/g)
mc = (lh*mh-p_eps_3/g)/lc
dh = (p_eps_2/g+sqrt(mc*(p_eps_1-lc^2*mc-lh^2*mh)))/mh
dc = (dh*mh-p_eps_2/g)/mc

%%
disp('Done with everything')

