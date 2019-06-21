function analyze_rotor_force_data()
c = Constants();
Kf = -2*c.l_h / (c.g * (c.m_c * c.l_c - 2* c.m_p * c.l_h));

rotor_force_data = evalin('base', 'rotor_force_data');
xs = rotor_force_data.v;
ys = cos(rotor_force_data.eps) / Kf;

param_guess = [1, 1];
opts = optimoptions(@lsqnonlin, 'SpecifyObjectiveGradient', true, 'FunctionTolerance', 1e-12);
param = lsqnonlin(@compute_error_vector, param_guess, [], [], opts);

p_opt = param(1)
q_opt = param(2)

figure
plot(xs, ys, 'x')
hold on
xs_plot = 0:0.01:1;
plot(xs_plot, Fr(xs_plot, p_opt, q_opt))

function [e, J] = compute_error_vector(param)
    n = numel(xs);
    e = zeros(n, 1);
    J = zeros(n, 2);
    
    p = param(1);
    q = param(2);
    
    for i = 1:n
        x = xs(i);
        if x <= 2 * q/p
            y = p^2/(4*q) * x^2;
            J(i, 1) = p/(4*q)*x^2;
            J(i, 2) = - p^2/(4*q^2) * x^2;
        else
            y = p * x - q;
            J(i, 1) = x;
            J(i, 2) = -1;
        end
        
        e(i) = y - ys(i);
    end
end

function out = Fr(xs_, p, q)
    out = zeros(numel(xs_), 1);
    
    for i=1:numel(xs_)
        x = xs_(i);
        
        if x <= 2 * q/p
            out(i) = p^2/(4*q) * x^2;
        else
            out(i) = p * x - q;
        end
    end
end
end