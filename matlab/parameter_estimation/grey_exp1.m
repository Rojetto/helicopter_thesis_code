function [ dx, y ] = grey_exp1( t, x, u, ps1, ps2, ps3, varargin)
p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;


Fd = Fr(u(1), p1, q1, p2, q2) - Fr(u(2), p1, q1, p2, q2);

phi = x(1);
dphi = x(2);

dx(1) = dphi;
dx(2) = (lp*Fd - ps2*dphi - ps3*sin(phi))/ps1;

y = phi;
end

