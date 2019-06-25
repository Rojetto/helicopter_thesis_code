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