c = Constants();
c_buildTapes_mex();

x780 = [Fr(vf(1, 1), c.p1, c.q1, c.p2, c.q2) + Fr(vb(1, 1), c.p1, c.q1, c.p2, c.q2); 0]

x_est0 = [phi(1, 1); eps(1, 1); lamb(1, 1); phi(1, 2); eps(1, 2); lamb(1, 2); x780]
z_est0 = c_calcPhi_mex(x_est0)

clear c