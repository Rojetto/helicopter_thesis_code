function [e, J] = compute_error_vector(param, xs, ys)
    n = numel(xs);
    e = zeros(n, 1);
    J = zeros(n, 2);
    
    p = param(1);
    q = param(2);
    d = param(3);
    
    for i = 1:n
        x = xs(i);
        if x <= 2 * q/p
            y = p^2/(4*q) * x^2 + d;
            J(i, 1) = p/(4*q)*x^2;
            J(i, 2) = - p^2/(4*q^2) * x^2;
            J(i, 3) = 1;
        else
            y = p * x - q + d;
            J(i, 1) = x;
            J(i, 2) = -1;
            J(i, 3) = 1;
        end
        
        e(i) = y - ys(i);
    end
end