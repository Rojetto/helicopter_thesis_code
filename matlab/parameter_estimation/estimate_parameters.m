p1 = 0.3117/2;
q1 = 0.9247/2;
p2 = 0.1396/2;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

sample_decimation = 10;

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
    [t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff(file_name, 0, 30);

    Ff = Fr(uf, p1, q1, p2, q2);
    Fb = Fr(ub, p1, q1, p2, q2);
    n = numel(t);


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
Ff = Fr(uf, p1, q1, p2, q2);
Fb = Fr(ub, p1, q1, p2, q2);
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
sys.Report.Parameters.ParVector(2)

disp('Done with everything')