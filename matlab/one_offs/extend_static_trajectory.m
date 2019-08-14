% traj data should be in workspace
samples = 500 * 10;
t = (0:samples-1)*0.002;

phi = repmat(phi, samples, 1);
eps = repmat(eps, samples, 1);
lamb = repmat(lamb, samples, 1);

vf = repmat(vf, samples, 1);
vb = repmat(vb, samples, 1);

clear samples