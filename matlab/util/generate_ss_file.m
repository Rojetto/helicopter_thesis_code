x = sym('x', [8 1]);
u = sym('u', [2 1]);

f = system_f(x, u);

A = jacobian(f, x);
B = jacobian(f, u);

matlabFunction(A, 'file', 'compute_A_full.m', 'vars', [x; u]);
matlabFunction(B, 'file', 'compute_B_full.m', 'vars', [x; u]);