function [ phi, gamma, lambda ] = c_calcPhiGammaLambda(x) %#codegen
setup_ad_build()

result = zeros(14, 1);
coder.cinclude('heli_adolc.h')
coder.ceval('calcRequiredDerivatives', coder.rref(x), coder.wref(result));

phi = [result(1:4); result(6:9)];
gamma = [result(5); result(10)];
lambda = [result(11:12) - gamma, result(13:14) - gamma];
end

