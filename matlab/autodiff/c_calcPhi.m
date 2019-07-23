function phi = c_calcPhi(x) %#codegen
setup_ad_build()

phi = zeros(8, 1);
coder.cinclude('heli_adolc.h')
coder.ceval('calcPhi', coder.rref(x), coder.wref(phi));
end

