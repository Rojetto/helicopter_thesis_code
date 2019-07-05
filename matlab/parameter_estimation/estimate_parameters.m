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

%% get phi parameters from step responses
files = {'exp6_open_step_neg_Vs0_Vd3_5',
'exp6_open_step_neg_Vs0_Vd5',
'exp6_open_step_neg_Vs4_Vd1',
'exp6_open_step_neg_Vs4_Vd2',
'exp6_open_step_pos_Vs0_Vd3_5',
'exp6_open_step_pos_Vs0_Vd5',
'exp6_open_step_pos_Vs4_Vd1',
'exp6_open_step_pos_Vs4_Vd2'};

start_times = [0, 0, 13, 13.5, 0, 0, 13, 11.6];
end_times = [4.2, 2.2, 17.8, 15.6, 5.2, 2.2, 17.45, 13.6];

[clips, data, inits] = make_experiment_clips(files, 'phi', ...
    'StartTimes', start_times, 'EndTimes', end_times, 'ClipLength', 8);

n_clips = numel(clips);

% add phi offset state that also has to be estimated
inits(end+1) = struct('Name', 'phi_off',...
    'Unit', 'rad',...
    'Value', zeros(1, n_clips),...
    'Minimum', deg2rad(-10)*ones(1, n_clips),...
    'Maximum', deg2rad(10)*ones(1, n_clips),...
    'Fixed', true(1, n_clips));

sys_init = idnlgrey('grey_exp4_only_phi', [1, 2, 3], [0.15; -0.007; mu_phi], inits);
sys_init.Parameters(2).Maximum = 0;
sys_init.Parameters(3).Fixed = true;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp6 phi parameters')

% resulting parameters
p_phi_1 = sys_est.Report.Parameters.ParVector(1);
p_phi_2 = sys_est.Report.Parameters.ParVector(2);

% for i=1:numel(files)
%     file_name = files{i};
%     disp(file_name)
%     [t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff(file_name);
%     
%     figure('Name', file_name)
%     subplot(211)
%     hold on
%     plot(t, phi)
%     plot(t, dphi)
%     plot(t, ddphi)
%     grid
%     
%     subplot(212)
%     hold on
%     plot(t, uf)
%     plot(t, ub)
%     grid
% end

%% get exp4 phi parameters
files = {'exp4_phi_feedback_rect_a5_f0_2_Vs4',
'exp4_phi_feedback_rect_a20_f0_2_Vs4',
'exp4_phi_feedback_rect_a20_f0_2_Vs6',
'exp4_phi_feedback_sine_a20_f0_2_Vs6',
'exp4_phi_feedback_sine_a45_f0_2_Vs6',
'exp4_phi_feedback_sine_a45_f0_4_Vs6'};

[clips, data, inits] = make_experiment_clips(files, 'phi');

n_clips = numel(clips);

% add phi offset state that also has to be estimated
inits(end+1) = struct('Name', 'phi_off',...
    'Unit', 'rad',...
    'Value', zeros(1, n_clips),...
    'Minimum', deg2rad(-10)*ones(1, n_clips),...
    'Maximum', deg2rad(10)*ones(1, n_clips),...
    'Fixed', false(1, n_clips));

sys_init = idnlgrey('grey_exp4_only_phi', [1, 2, 3], [0.15; -0.007; mu_phi], inits);
sys_init.Parameters(3).Fixed = true;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp4 phi parameters')

% resulting parameters
p_phi_1 = sys_est.Report.Parameters.ParVector(1);
p_phi_2 = sys_est.Report.Parameters.ParVector(2);

%% exp4 lambda parameters
files = {'exp4_lamb_feedback_rect_a10_f0_05',
'exp4_lamb_feedback_sine_a20_f0_1',
'exp4_lamb_feedback_sine_a20_f0_2_Vs7',
'exp4_lamb_open_step_phi_neg',
'exp4_lamb_open_step_phi_pos'};

start_times = [0, 0, 0, 7, 7];

[clips, data, inits] = make_experiment_clips(files, 'lamb',...
    'InputVariables', {'uf', 'ub', 'phi'},...
    'StartTimes', start_times);
n_clips = numel(clips);

inits(end+1) = struct('Name', 'phi_off',...
    'Unit', 'rad',...
    'Value', zeros(1, n_clips),...
    'Minimum', deg2rad(-10)*ones(1, n_clips),...
    'Maximum', deg2rad(10)*ones(1, n_clips),...
    'Fixed', false(1, n_clips));

sys_init = idnlgrey('grey_exp4_only_lamb', [1, 3, 3], [1.28, 0.0466], inits);

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
plot_meas_and_fit(sys_est, data, 'Exp4 lambda parameters')

% resulting parameters
p_lamb_1 = sys_est.Report.Parameters.ParVector(1);
mu_lamb = sys_est.Report.Parameters.ParVector(2);

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

