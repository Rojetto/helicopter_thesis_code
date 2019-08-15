four_poles = false;

if four_poles
p = [-4 -4 -4 -4];
else
p = [-3 -3 -3];
end

syms x
if four_poles
poly = expand((x - p(1))*(x - p(2))*(x - p(3))*(x - p(4)));
else
poly = expand((x - p(1))*(x - p(2))*(x - p(3)));
end
poly_coeffs = coeffs(poly);

k0 = double(poly_coeffs(1))
k1 = double(poly_coeffs(2))
k2 = double(poly_coeffs(3))
if four_poles
k3 = double(poly_coeffs(4))
end

%sim pole_placement_test

syms y(t_symb)
d1y = diff(y);
d2y = diff(y, t_symb, 2);
d3y = diff(y, t_symb, 3);
if four_poles
d4y = diff(y, t_symb, 4);
end

if four_poles
ode = d4y + k3*d3y + k2*d2y + k1*d1y + k0*(y-1)==0;
else
ode = d3y + k2*d2y + k1*d1y + k0*(y-1)==0;
end
cond1 = y(0) == 0;
cond2 = d1y(0) == 0;
cond3 = d2y(0) == 0;
if four_poles
cond4 = d3y(0) == 0;
end

if four_poles
ySol(t_symb) = dsolve(ode, [cond1 cond2 cond3 cond4])
else
ySol(t_symb) = dsolve(ode, [cond1 cond2 cond3])
end

t_plot = 0:0.1:10;
% figure
% plot(t_plot, ySol(t_plot))
% grid

if four_poles
disp(mat2str([k3, 0; k2, 0; k1, 0; k0, 0; 0, k3; 0, k2; 0, k1; 0, k0]))
end