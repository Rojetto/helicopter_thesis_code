function [ dx, y ] = grey_exp0( t, x, u,...
                                p_phi_1, p_phi_2, mu_phi,...
                                p_eps_1, p_eps_2, p_eps_3, mu_eps,...
                                p_lamb_1, mu_lamb, varargin)
p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

Ff = Fr(u(1), p1, q1, p2, q2);
Fb = Fr(u(2), p1, q1, p2, q2);
Fs = Ff + Fb;
Fd = Ff - Fb;

phi = x(1);
dphi = x(2);

eps = x(3);
deps = x(4);

lamb = x(5);
dlamb = x(6);

dx(1) = dphi;
dx(2) = (lp*Fd - mu_phi*dphi-p_phi_2*sin(phi))/p_phi_1;

dx(3) = deps;
dx(4) = (lh*cos(phi)*Fs-mu_eps*deps-p_eps_2*sin(eps)-p_eps_3*cos(eps))/p_eps_1;

dx(5) = dlamb;
dx(6) = (lh*sin(phi)*cos(eps)*Fs-lp*sin(eps)*Fd-mu_lamb*dlamb)/p_lamb_1;

y = [phi; eps; lamb];
end

