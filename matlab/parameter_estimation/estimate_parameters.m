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
clip_size = 7; % s
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
        
        clips{end+1} = struct('t', t_clip', 'eps', eps_clip, 'deps', deps_clip, 'uf', uf_clip, 'ub', ub_clip);
        
        i_start = i_end + 1;
    end
end

exp5_clip_ps = zeros(4,0);

% fit clips
for i=1:numel(clips)
    clip = clips{i};

    init_params = [0.23, 0.37, 0.215, mu_eps];
    init_states = [clip.eps(1); clip.deps(1)];
    
    % define grey box model
    id_sys = idnlgrey('grey_exp5', [1, 2, 2], init_params, init_states);
    id_sys.Parameters(1).Minimum = 0;
    id_sys.Parameters(2).Minimum = 0;
    id_sys.Parameters(3).Minimum = 0;
    id_sys.Parameters(4).Minimum = 0;
    id_data = iddata(clip.eps, [clip.uf, clip.ub], 0.002);
    id_data = resample(id_data, 1, sample_decimation);


    % estimate model parameters
    sys = nlgreyest(id_data, id_sys);
    
    clips{i}.fitPercent = sys.Report.Fit.FitPercent;
    clips{i}.fitMSE = sys.Report.Fit.MSE;
    clips{i}.fitParams = sys.Report.Parameters.ParVector;

    exp5_clip_ps(:, end+1) = sys.Report.Parameters.ParVector;

    plot_meas_and_fit(sys, id_data, file_name)
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


%%
disp('Done with everything')

