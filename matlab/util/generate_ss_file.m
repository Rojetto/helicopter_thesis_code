x8states = sym('x', [8 1]);
x6states = x8states(1:6);
u = sym('u', [2 1]);

f = system_f(x8states, u);

A8states = jacobian(f, x8states);
B8states = jacobian(f, u);

A6states = A8states(1:6,1:6);
B6states = B8states(1:6,:);

%matlabFunction(A8states, 'file', 'compute_A_8_states.m', 'vars', [x8states; u]);
%matlabFunction(B8states, 'file', 'compute_B_8_states.m', 'vars', [x8states; u]);

matlabFunction(A6states, 'file', 'compute_A_6_states.m', 'vars', [x6states; u]);
matlabFunction(B6states, 'file', 'compute_B_6_states.m', 'vars', [x6states; u]);