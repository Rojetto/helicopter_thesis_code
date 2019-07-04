function [ dx, y ] = grey_exp5( t, x, u, p_eps_1, p_eps_2, p_eps_3, mu_eps, varargin)
p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;


Fs = Fr(u(1), p1, q1, p2, q2) + Fr(u(2), p1, q1, p2, q2);

eps = x(1);
deps = x(2);

dx(1) = deps;
dx(2) = (lh*Fs - mu_eps*deps - p_eps_2*sin(eps) - p_eps_3*cos(eps))/p_eps_1;

y = eps;
end

