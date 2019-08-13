c_buildTapes_mex()

x = [1 2 3 4 5 6 7 8]';

tic
for i=1:1000
    phi = c_calcPhi_mex(x);
    gamma = c_calcGamma_mex(x);
    lambda = c_calcLambda_mex(x);
end
toc

tic
for i=1:1000
    [phi, gamma, lambda] = c_calcPhiGammaLambda_mex(x);
end
toc