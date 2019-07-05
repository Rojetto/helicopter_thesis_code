function [ dx, y ] = grey_exp4_only_phi( t, x, u, p_phi_1, p_phi_2, mu_phi, varargin)
p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;


Fd = Fr(u(1), p1, q1, p2, q2) - Fr(u(2), p1, q1, p2, q2);

phi = x(1);
dphi = x(2);
phi_off = x(3);

dx(1) = dphi;
dx(2) = (lp*Fd - mu_phi*dphi - p_phi_2*sin(phi - phi_off))/p_phi_1;
dx(3) = 0;

y = phi;
end

