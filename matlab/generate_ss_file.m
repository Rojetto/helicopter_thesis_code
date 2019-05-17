x = sym('x', [6 1]);
u = sym('u', [2 1]);

vs = (u(1) + u(2))/2;
vd = (u(1) - u(2))/2;

c = Constants();

f = [
    x(4);
    x(5);
    x(6);
    1/c.Jp * ( c.L1 * vd - c.mup*x(4) );
    1/c.Je * ( c.L2*cos(x(2)) + c.L3 * vs * cos(x(1)) - c.mue*x(5) );
    1/c.Jl * ( c.L4 * vs * cos(x(2)) * sin(x(1)) - c.mul*x(6) )
];

A = jacobian(f, x);
B = jacobian(f, u);

matlabFunction(A, 'file', 'compute_A.m', 'vars', [x; u]);
matlabFunction(B, 'file', 'compute_B.m', 'vars', [x; u]);