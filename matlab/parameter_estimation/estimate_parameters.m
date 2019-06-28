p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

%% get phi parameters for exp1

%[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_open_rect_a1_f0_3');
%[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_open_rect_a0_5_f0_2');
%[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_open_sine_a0_5_f0_2');
%[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_open_sine_a0_5_f0_2_higher_vs');
%[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_open_sine_a1_f0_2');
%[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_feedback_sine_a25_f0_3');
%[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_feedback_sine_a40_f0_2');
[t, phi, dphi, ddphi, ~, ~, ~, ~, ~, ~, uf, ub] = load_and_diff('exp1_feedback_sine_a40_f0_3', 0, 10);

Ff = Fr(uf, p1, q1, p2, q2);
Fb = Fr(ub, p1, q1, p2, q2);
n = numel(t);

A = zeros(n, 3);
b = zeros(n, 1);
A(:,1) = ddphi;
A(:,2) = dphi;
A(:,3) = sin(phi);

b(:) = lp.*(Ff-Fb);

p = (A'*A) \ (A'*b)


%% using nlgreybox
id_sys = idnlgrey('grey_exp1', [1, 2, 2], [0.2; 0.01; 0.4], [0; 0]);
id_data = iddata([phi], [uf, ub], 0.002);

sys = nlgreyest(id_data, id_sys);