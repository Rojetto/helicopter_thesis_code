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


%% get exp5 eps parameters
exp5_files = {'exp5_feedback_rect_a5_f0_1',
'exp5_feedback_rect_a10_f0_1',
'exp5_feedback_sine_a10_f0_2',
'exp5_feedback_sine_a10_f0_4',
'exp5_open_rect_a0_8_f0_05_off7',
'exp5_open_rect_a1_f0_05_off6',
'exp5_open_sine_a0_1_f0_2_off7',
'exp5_open_sine_a0_5_f0_2_off7'};

t0s = [4, 4, 4, 4, 12, 4, 15, 7];

exp5_ps = zeros(4, 0);
exp5_reports = {};

for i=1:numel(exp5_files)
    file_name = exp5_files{i};
    [t, ~, ~, ~, eps, deps, ~, ~, ~, ~, uf, ub] = load_and_diff(file_name, t0s(i));

    init_params = [0.23, 0.37, 0.215, mu_eps];
    init_states = [eps(1); deps(1)];
    
    % define grey box model
    id_sys = idnlgrey('grey_exp5', [1, 2, 2], init_params, init_states);
    id_sys.Parameters(1).Minimum = 0;
    id_sys.Parameters(2).Minimum = 0;
    id_sys.Parameters(3).Minimum = 0;
    id_sys.Parameters(4).Minimum = 0;
    id_data = iddata([eps], [uf, ub], 0.002);
    id_data = resample(id_data, 1, sample_decimation);


    % estimate model parameters
    sys = nlgreyest(id_data, id_sys);

    exp5_ps(:, end+1) = sys.Report.Parameters.ParVector;
    exp5_reports{end+1} = sys.Report;

    plot_meas_and_fit(sys, id_data, file_name)
end

%% try splitting exp5 data into small clips
clip_size = 10; % s
clip_size_samples = clip_size * 500;

exp5_files = {'exp5_feedback_sine_a10_f0_2',
'exp5_feedback_sine_a10_f0_4',
'exp5_open_rect_a0_8_f0_05_off7',
'exp5_open_rect_a1_f0_05_off6',
'exp5_open_sine_a0_1_f0_2_off7',
'exp5_open_sine_a0_5_f0_2_off7'};

clips = {};

% cut into clips
for i=1:numel(exp5_files)
    file_name = exp5_files{i};
    [t, ~, ~, ~, eps, deps, ~, ~, ~, ~, uf, ub] = load_and_diff(file_name);
    
    i_start = find(eps > deg2rad(-25), 1);
    
    while i_start < numel(t)
        i_end = min(i_start + clip_size_samples - 1, numel(t));
        
        samples_clip = i_end - i_start + 1;
        t_clip = (0:(samples_clip-1))*0.002;
        eps_clip = eps(i_start:i_end);
        deps_clip = deps(i_start:i_end);
        uf_clip = uf(i_start:i_end);
        ub_clip = ub(i_start:i_end);
        
        clips{end+1} = struct('t', t_clip', 'eps', eps_clip,...
            'deps', deps_clip, 'uf', uf_clip, 'ub', ub_clip,...
            'file', file_name, 'tStart', i_start*0.002, 'tEnd', i_end*0.002);
        
        i_start = i_end + 1;
    end
end

exp5_clip_ps = zeros(4,0);

% fit clips
for i=1:numel(clips)
    clip = clips{i};

    init_params = [0.9, 1.0699, 0.8375, mu_eps];
    init_states = [clip.eps(1); clip.deps(1)];
    
    % define grey box model
    id_sys = idnlgrey('grey_exp5', [1, 2, 2], init_params, init_states);
    id_sys.Parameters(1).Minimum = 0.0;
    id_sys.Parameters(1).Maximum = 1.4;
    %id_sys.Parameters(2).Minimum = 0.8;
    %id_sys.Parameters(2).Maximum = 2.0;
    id_sys.Parameters(2).Fixed = true;
    %id_sys.Parameters(3).Minimum = 0.8;
    %id_sys.Parameters(3).Maximum = 0.9;
    id_sys.Parameters(3).Fixed = true;
    id_sys.Parameters(4).Minimum = 0.01;
    id_sys.Parameters(4).Maximum = 0.09;
    id_data = iddata(clip.eps, [clip.uf, clip.ub], 0.002);
    id_data = resample(id_data, 1, sample_decimation);
    
    opt = nlgreyestOptions;
    opt.Display = 'on';


    % estimate model parameters
    sys = nlgreyest(id_data, id_sys, opt);
    
    clips{i}.fitPercent = sys.Report.Fit.FitPercent;
    clips{i}.fitMSE = sys.Report.Fit.MSE;
    clips{i}.fitParams = sys.Report.Parameters.ParVector;

    exp5_clip_ps(:, end+1) = sys.Report.Parameters.ParVector;

    plot_meas_and_fit(sys, id_data, clips{i}.file)
end

% average estimated parameters, weighted by inverse of fit MSE
sum_weight = 0;
sum_weighted_params = 0;
for i=1:numel(clips)
    fitParams = clips{i}.fitParams;
    weight = 1/clips{i}.fitMSE;
    
    sum_weight = sum_weight + weight;
    sum_weighted_params = sum_weighted_params + weight*fitParams;
end

sum_weighted_params / sum_weight

%%
figure
hold on
for i=1:numel(clips)
    clip = clips{i};

    init_params = median_ps;
    init_states = [clip.eps(1); clip.deps(1)];
    
    % define grey box model
    id_sys = idnlgrey('grey_exp5', [1, 2, 2], init_params, init_states);
    id_data = iddata(clip.eps, [clip.uf, clip.ub], 0.002);

    plot_meas_and_fit(id_sys, id_data, file_name)
end

%% p_eps_2 and p_eps_3 from static elevation steps

[t, ~, ~, ~, eps, deps, ddeps, ~, ~, ~, uf, ub] = load_and_diff('elevation_steps');

start_times = (11.2:3:161.2);
end_times = start_times + 0.3;

sections = [start_times', end_times'];
n_datapoints = size(sections, 1);

avg_eps = zeros(n_datapoints, 1);        
avg_vf = zeros(n_datapoints, 1);
avg_vb = zeros(n_datapoints, 1);

all_indices = zeros(numel(t), 1);

for i=1:n_datapoints
    indices = t > sections(i,1) & t < sections(i,2);
    all_indices = all_indices | indices;
    avg_eps(i) = mean(eps(indices));
    avg_uf(i) = mean(uf(indices));
    avg_ub(i) = mean(ub(indices));
end

avg_us = avg_uf + avg_ub;
Fs = Fr(avg_uf, p1, q1, p2, q2) + Fr(avg_ub, p1, q1, p2, q2);

figure
hold on
static_points = abs(ddeps) < 0.05 & abs(deps) < 0.022 & t > 11 & t < 160;
plot(t, eps)
plot(t(all_indices), eps(all_indices), 'x')
plot((start_times + end_times)/2, avg_eps, 'o')
plot((start_times + end_times)/2, 0.5*(avg_uf + avg_ub), 'o')
plot((start_times + end_times)/2, Fs, 'o')
grid

A = [sin(avg_eps), cos(avg_eps)];
b = lh*Fs;

abc = (A'*A) \ (A'*b)

figure
hold on
plot(avg_eps, b, 'x');
plot(avg_eps, A*abc);

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

