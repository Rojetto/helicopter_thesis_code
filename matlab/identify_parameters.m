function identify_parameters()
close all
h = 0.002;
numdiff_test_data = evalin('base', 'numdiff_test_data');
t = numdiff_test_data.getElement('meas_phi').Values.Time(2:end-1);
vf = numdiff_test_data.getElement('vf').Values.Data;
vf = reshape(vf, numel(vf), 1);
vf = vf(2:end-1);
vb = numdiff_test_data.getElement('vb').Values.Data;
vb = reshape(vb, numel(vb), 1);
vb = vb(2:end-1);
phi = numdiff_test_data.getElement('meas_phi').Values.Data;
eps = numdiff_test_data.getElement('meas_eps').Values.Data;
lamb = numdiff_test_data.getElement('meas_lamb').Values.Data;
dphi = numdiff(phi, h, 1, 1);
deps = numdiff(eps, h, 1, 1);
dlamb = numdiff(lamb, h, 1, 1);
ddphi = numdiff(phi, h, 2, 1);
ddeps = numdiff(eps, h, 2, 1);
ddlamb = numdiff(lamb, h, 2, 1);

phi = phi(2:end-1);
eps = eps(2:end-1);
lamb = lamb(2:end-1);

% Trimming
start_i = 1;
end_i = 1000;
t = t(start_i:end_i);
vf = vf(start_i:end_i);
vb = vb(start_i:end_i);
phi = phi(start_i:end_i);
eps = eps(start_i:end_i);
lamb = lamb(start_i:end_i);
dphi = dphi(start_i:end_i);
deps = deps(start_i:end_i);
dlamb = dlamb(start_i:end_i);
ddphi = ddphi(start_i:end_i);
ddeps = ddeps(start_i:end_i);
ddlamb = ddlamb(start_i:end_i);

warning('off', 'Control:analysis:LsimStartTime')

param_guess = [1.0, 0.5, 0.5, 0.3, 0.7, 0.1, 0.1, 0.1, 0.2, 1.9516e-5, 0.2];
lb = [0.5, 0.1, 0.2, 0.1, 0.5, 0.01, 0.01, 0.01, 0.001, 0, 0.01];
ub = [2, 1, 0.5, 0.5, 1, 0.5, 0.5, 0.5, 0.2, 0.01, 0.3];
opt_params = lsqnonlin(@compute_error_vector, param_guess, lb, ub)
assignin('base', 'opt_params_lsq', opt_params)
opt_params = ga(@(v) sum(compute_error_vector(v).^2), 11, [], [], [], [], lb, ub)
assignin('base', 'opt_params_ga', opt_params)

function e = compute_error_vector(params)
    n = numel(ddphi);
    e = zeros(n*3, 1);

    m_c = params(1);
    m_p = params(2);
    
    l_c = params(3);
    l_p = params(4);
    l_h = params(5);

    mup = params(6);
    mue = params(7);
    mul = params(8);
    
    T_w = params(9);
    K_w = 1;

    p1 = 1.0;
    q1 = 0.5;
    p2 = 0.6;
    q2 = 0.3;

    J_m = params(10);
    K_m = params(11);

    motor_pt1 = tf(1, [T_w, 1]);
    wf = lsim(motor_pt1, vf, t);
    wb = lsim(motor_pt1, vb, t);

    for i=1:n
        x_est = [phi(i), eps(i), lamb(i), dphi(i), deps(i), dlamb(i), wf(i), wb(i)];
        u_meas = [vf(i), vb(i)];
        dx_model = system_f_with_params(x_est, u_meas, ...
            m_c, m_p, l_c, l_p, l_h, mup, mue, mul, ...
            T_w, K_w, p1, q1, p2, q2, J_m, K_m);

        e(3*(i-1) + 1) = ddphi(i) - dx_model(4);
        e(3*(i-1) + 2) = ddeps(i) - dx_model(5);
        e(3*(i-1) + 3) = ddlamb(i) - dx_model(6);
    end
end

function diff_data = numdiff(data, h, order, accuracy)
    if order == 1
        if accuracy == 1
            numerator = -1/2*data(1:end-2) + 1/2*data(3:end);
        elseif accuracy == 2
            numerator = 1/12*data(1:end-4) - 2/3*data(2:end-3) + 2/3*data(4:end-1) - 1/12*data(5:end);
        end
        
        diff_data = numerator / h;
    elseif order == 2
        if accuracy == 1
            numerator = 1*data(1:end-2) - 2*data(2:end-1) + 1*data(3:end);
        elseif accuracy == 2
            numerator = -1/12*data(1:end-4) + 4/3*data(2:end-3) - 5/2*data(3:end-2) + 4/3*data(4:end-1) - 1/12*data(5:end);
        end
        
        diff_data = numerator / h^2;
    end
end
end