function [cost, jac] = model_param_solution_cost(model_params, p_phi_1, p_phi_2, p_eps_1, p_eps_2, p_eps_3, p_lamb_1)
lp = 0.178;
lh = 0.67;
g = 9.81;

dh = model_params(1);
mh = model_params(2);
lc = model_params(3);
dc = model_params(4);
mc = model_params(5);

cost(1) = (dh^2+lp^2)*mh - p_phi_1;
cost(2) = -dh*mh*g - p_phi_2;
cost(3) = (lh^2+dh^2)*mh+(lc^2+dc^2)*mc - p_eps_1;
cost(4) = g*(dc*mc-dh*mh) - p_eps_2;
cost(5) = g*(lh*mh-lc*mc) - p_eps_3;
cost(6) = lc^2*mc+(lh^2+lp^2)*mh - p_lamb_1;

jac = [2*dh*mh, (dh^2+lp^2), 0, 0, 0;
       -mh*g, -dh*g, 0, 0, 0;
       2*dh*mh, lh^2+dh^2, 2*lc*mc, 2*dc*mc, lc^2+dc^2;
       -g*mh, -g*dh, 0, g*mc, g*dc;
       0, g*lh, -g*mc, 0, -g*lc;
       0, lh^2+lp^2, 2*lc*mc, 0, lc^2];

end

