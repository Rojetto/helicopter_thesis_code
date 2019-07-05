function [ dx, y ] = grey_exp4_only_lamb( t, x, u, p_lamb_1, mu_lamb, varargin )
p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

uf = u(1);
ub = u(2);
phi = u(3);

Ff = Fr(uf, p1, q1, p2, q2);
Fb = Fr(ub, p1, q1, p2, q2);

Fs = Ff + Fb;
Fd = Ff - Fb;

lamb = x(1);
dlamb = x(2);
phi_off = x(3);

dx(1) = dlamb;
dx(2) = (lh*sin(phi - phi_off)*Fs - mu_lamb*dlamb)/p_lamb_1;
dx(3) = 0;

y = lamb;
end

