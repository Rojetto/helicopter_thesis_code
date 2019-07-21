function gamma = c_calcGamma(x) %#codegen
setup_ad_build()

gamma = zeros(2, 1);
coder.cinclude('heli_adolc.h')
coder.ceval('calcGamma', coder.rref(x), coder.wref(gamma));
end

