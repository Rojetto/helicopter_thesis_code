function x = phi_inv(z, x_guess)
step_tol = 1e-9;

if nargin==1
    x_guess = [0; 0; -0.2; 0.02; 0; 0; 0; 0];
end

x = x_guess;

while true
    if coder.target('MATLAB')
        phi = c_calcPhi_mex(x);
        phi_jac = c_calcPhiJacobian_mex(x);
    else
        phi = c_calcPhi(x);
        phi_jac = c_calcPhiJacobian(x);
    end
    
    dx = phi_jac \ (z - phi);
    
    x = x + dx;
    
    if max(abs(dx)) < step_tol
        break
    end
end
end

