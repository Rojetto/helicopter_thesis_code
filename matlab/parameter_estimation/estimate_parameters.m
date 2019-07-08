p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

g = 9.81;

sample_decimation = 20;

do_plots = true;

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
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp1 Parameters except friction')
end

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
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp1 mu_phi')
end

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
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp3 Parameters except friction')
end

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
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp3 mu_eps')
end

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
    'StartTimes', start_times, 'ResamplingDecimation', 20);

sys_init = idnlgrey('grey_exp5', [1, 2, 2], [0.838; 1.33; 0.838; mu_eps], inits);
sys_init.Parameters(4).Fixed = true;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp5 p_eps_1')
end

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
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp5 p_eps_2 and p_eps_3')
end

% resulting parameters
p_eps_2 = sys_est.Report.Parameters.ParVector(2);
p_eps_3 = sys_est.Report.Parameters.ParVector(3);

%% get phi parameters from step responses
files = {'exp6_open_step_neg_Vs4_Vd1',
'exp6_open_step_neg_Vs4_Vd2',
'exp6_open_step_pos_Vs4_Vd1',
'exp6_open_step_pos_Vs4_Vd2'};

start_times = [13, 13.5, 13, 11.6];
end_times = [17.8, 15.6, 17.45, 13.6];

[clips, data, inits] = make_experiment_clips(files, 'phi', ...
    'StartTimes', start_times, 'EndTimes', end_times, 'ClipLength', 8);

n_clips = numel(clips);

% add phi offset state
inits(end+1) = struct('Name', 'phi_off',...
    'Unit', 'rad',...
    'Value', zeros(1, n_clips),...
    'Minimum', deg2rad(-10)*ones(1, n_clips),...
    'Maximum', deg2rad(10)*ones(1, n_clips),...
    'Fixed', true(1, n_clips));

sys_init = idnlgrey('grey_exp4_only_phi', [1, 2, 3], [0.04; -0.007; mu_phi], inits);
sys_init.Parameters(2).Maximum = 0;
sys_init.Parameters(3).Fixed = false;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp6 phi parameters')
end

% resulting parameters
p_phi_1 = sys_est.Report.Parameters.ParVector(1);
p_phi_2 = sys_est.Report.Parameters.ParVector(2);

%% calculate physical model parameters
lp
lh
dh = -g*p_phi_1/(2*p_phi_2) - sqrt((g*p_phi_1)^2/(4*p_phi_2^2) - lp^2)
mh = -p_phi_2/(dh*g)

%% lc, dc, mc from p_eps_1, p_eps_2, p_eps_e
intermediate1 = ((dh*g*mh)^2+2*dh*g*mh*p_eps_2+(g*lh*mh)^2-2*g*lh*mh*p_eps_3+p_eps_2^2+p_eps_3^2);
intermediate2 = (dh^2*mh+lh^2*mh-p_eps_1);
lc = -g*(g*lh*mh-p_eps_3)*intermediate2/intermediate1
dc = -g*(dh*g*mh+p_eps_2)*intermediate2/intermediate1
mc = -intermediate1/(g^2*intermediate2)

%% exp4 lambda parameters
p_lamb_1 = mh*(lh^2+lp^2) + mc*lc^2;

files = {'exp4_lamb_feedback_rect_a10_f0_05',
'exp4_lamb_feedback_sine_a20_f0_1',
'exp4_lamb_feedback_sine_a20_f0_2_Vs7',
%'exp4_lamb_open_step_phi_neg',
%'exp4_lamb_open_step_phi_pos'
};

start_times = [0, 0, 0];
%start_times = [7, 7];
%start_times = [0, 0, 0, 7, 7];

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

sys_init = idnlgrey('grey_exp4_only_lamb', [1, 3, 3], [p_lamb_1, 0.25], inits);
sys_init.Parameters(1).Fixed = true;

% estimate model parameters
sys_est = nlgreyest(data, sys_init, 'Display', 'on');

% plot
if do_plots
    plot_meas_and_fit(sys_est, data, 'Exp4 lambda parameters')
end

% resulting parameters
%p_lamb_1 = sys_est.Report.Parameters.ParVector(1);
mu_lamb = sys_est.Report.Parameters.ParVector(2);

%% lc, dc, mc from p_lamb_1, p_eps_2, p_eps_e
i1 = (lh^2+lp^2)*mh-p_lamb_1;
i2 = g*lh*mh-p_eps_3;

%lc = -g*i1/i2
%dc = -g*(dh*g*mh+p_eps_2)*i1/i2^2
%mc = -i2^2/(g^2*i1)

%% try simulating full system with all parameters
[clips, data, inits] = make_experiment_clips({'exp0_sine_elevation'}, ...
    {'phi', 'eps', 'lamb'}, 'StartTimes', 5, 'ClipLength', 120);

params = [p_phi_1, p_phi_2, mu_phi,...
          p_eps_1, p_eps_2, p_eps_3, mu_eps,...
          p_lamb_1, mu_lamb];
sys_init = idnlgrey('grey_exp0', [3, 2, 6], params, inits);

plot_meas_and_fit(sys_init, data);
%%
min_fun = @(p) model_param_solution_cost(p, p_phi_1, p_phi_2, p_eps_1, p_eps_2, p_eps_3, p_lamb_1);
opts = optimoptions('lsqnonlin', 'Jacobian', 'on', 'MaxIter', 1000);
%result = lsqnonlin(min_fun, [dh;mh;lc;dc;mc], [], [], opts)
%%
disp('Done with everything')

