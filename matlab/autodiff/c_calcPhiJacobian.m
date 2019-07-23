function phi_jac = c_calcPhiJacobian(x) %#codegen
setup_ad_build()

phi_jac_flat = zeros(8*8, 1);
phi_jac = zeros(8, 8);
coder.cinclude('heli_adolc.h')
coder.ceval('calcPhiJacobian', coder.rref(x), coder.wref(phi_jac_flat));
phi_jac(:) = phi_jac_flat;
end
