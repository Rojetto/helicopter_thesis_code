function out = Fr_inv(F, p1, q1, p2, q2)
    out = zeros(numel(F), 1);
    
    for i=1:numel(F)
        y = F(i);
        
        if y <= - q2
            out(i) = (y - q2) / p2;
        elseif y < 0
            out(i) = - sqrt(-4*q2*y) / p2;
        elseif y < q1
            out(i) = sqrt(4*q1*y) / p1;
        else
            out(i) = (y + q1) / p1;
        end
    end
end