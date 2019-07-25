p = [-10 -10 -10 -10];

syms x
poly = expand((x - p(1))*(x - p(2))*(x - p(3))*(x - p(4)));
poly_coeffs = coeffs(poly);

k0 = double(poly_coeffs(1))
k1 = double(poly_coeffs(2))
k2 = double(poly_coeffs(3))
k3 = double(poly_coeffs(4))

sim pole_placement_test

syms y(t)
d1y = diff(y);
d2y = diff(y, t, 2);
d3y = diff(y, t, 3);
d4y = diff(y, t, 4);

ode = d4y + k3*d3y + k2*d2y + k1*d1y + k0*(y-1)==0;
cond1 = y(0) == 0;
cond2 = d1y(0) == 0;
cond3 = d2y(0) == 0;
cond4 = d3y(0) == 0;

ySol(t) = dsolve(ode, [cond1 cond2 cond3 cond4])