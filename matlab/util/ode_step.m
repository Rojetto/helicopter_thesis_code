function x_next = ode_step(f, x_last, u, h, params)
x_next = x_last + h*f(x_last, u, params);
end

