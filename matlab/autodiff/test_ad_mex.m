x = [1 2 3 4 5 6 7 8]';

c_buildTapes_mex();

gamma = c_calcGamma_mex(x)
lambda = c_calcLambda_mex(x)
phi = c_calcPhi_mex(x)
phi_jac = c_calcPhiJacobian_mex(x)

[phi_alt, gamma_alt, lambda_alt] = c_calcPhiGammaLambda_mex(x)