function out = Fr(u, p1, q1, p2, q2)
    out = zeros(numel(u), 1);
    
    for i=1:numel(u)
        x = u(i);
        
        if x <= -2*q2/p2
            out(i) = p2*x + q2;
        elseif x <= 0
            out(i) = - p2^2/(4*q2)*x^2;
        elseif x <= 2 * q1/p1
            out(i) = p1^2/(4*q1) * x^2;
        else
            out(i) = p1 * x - q1;
        end
    end
end