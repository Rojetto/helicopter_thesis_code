function result = integrate_matrix(fun, t_start, t_end, steps)
    ts = linspace(t_start, t_end, steps);
    x0 = fun(t_start);
    size_x = size(x0);
    xs = zeros(numel(ts), size_x(1), size_x(2));
    
    for i=1:numel(ts)
        xs(i,:,:) = fun(ts(i));
    end
    
    Q = trapz(ts, xs, 1);
    result = reshape(Q(1, :, :), 8, 8);
end

