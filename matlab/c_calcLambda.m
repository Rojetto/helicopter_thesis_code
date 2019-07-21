function lambda = c_calcLambda(x) %#codegen
setup_ad_build()

lambda_flat = zeros(4, 1);
lambda = zeros(2, 2);
coder.cinclude('heli_adolc.h')
coder.ceval('calcLambda', coder.rref(x), coder.wref(lambda_flat));
lambda(:) = lambda_flat;
end
