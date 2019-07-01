function [ dx, y ] = grey_exp4_only_phi( t, x, u, p_phi_1, p_phi_2, mu_phi, phi_off, varargin)
p1 = 0.3117/2;
q1 = 0.9247/2;
p2 = 0.1396/2;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;


Fd = Fr(u(1), p1, q1, p2, q2) - Fr(u(2), p1, q1, p2, q2);

phi = x(1);
dphi = x(2);

dx(1) = dphi;
dx(2) = (lp*Fd - p_phi_2*dphi - mu_phi*sin(phi - phi_off))/p_phi_1;

y = phi;
end

