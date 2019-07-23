function x_next = ode_step(f, x_last, u, h)
x_next = x_last + h*f(x_last, u);
end

