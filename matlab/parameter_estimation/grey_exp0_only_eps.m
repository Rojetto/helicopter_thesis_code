function [ dx, y ] = grey_exp0_only_eps( t, x, u, ps1, mu_eps, ps2, ps3, varargin)
p1 = 0.3117/2;
q1 = 0.9247/2;
p2 = 0.1396/2;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;


Fs = Fr(u(1), p1, q1, p2, q2) + Fr(u(2), p1, q1, p2, q2);

eps = x(1);
deps = x(2);

dx(1) = deps;
dx(2) = (lh*Fs - mu_eps*deps - ps2*sin(eps) - ps3*cos(eps))/ps1;

y = eps;
end

