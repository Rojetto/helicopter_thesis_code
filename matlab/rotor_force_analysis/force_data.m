function force_data()
% Sum of voltages in V
Vs_high = [3.5, 4, 4.5, 5, 5.5, 6, 6.5, 7, 7.5, 8, 8.5, 9, 9.5];
Vs_low = [2, 2.5, 3, 3.5, 4, 4.5];
% Upwards force in g
Fs_high = [35, 45, 55, 67, 80, 95, 110, 125, 140, 155, 170, 180, 180] / 100;
Fs_low = [10, 14, 22, 30, 42, 52] / 100;

Vs_combined = [Vs_low, Vs_high(4:end-2)];
Fs_combined = [Fs_low, Fs_high(4:end-2)];

xs = Vs_combined;
ys = Fs_combined;

param_guess = [1, 1];
opts = optimoptions('lsqnonlin', 'Jacobian', 'on', 'TolFun', 1e-12);
param = lsqnonlin(@compute_error_vector, param_guess, [], [], opts);

p_opt = param(1)
q_opt = param(2)

Vs_plot = 0:0.1:10;

figure(1)
hold on
plot(Vs_high, Fs_high, 'bx')
plot(Vs_low, Fs_low, 'rx')
plot(Vs_combined, Fs_combined)
plot(Vs_plot, Fs(Vs_plot, p_opt, q_opt))
xlim([0, 10])
ylim([0, 2])
xlabel('Vs [V]')
ylabel('Fs [g]')

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

function out = Fs(Vs, p, q)
    out = zeros(numel(Vs), 1);
    
    for i=1:numel(Vs)
        x = Vs(i);
        
        if x <= 2 * q/p
            out(i) = p^2/(4*q) * x^2;
        else
            out(i) = p * x - q;
        end
    end
end
end