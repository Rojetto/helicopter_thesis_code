Q_diag = [2, 10, 4, 0.2, 0.2, 0.1];
R_diag = [0.1, 0.1];


Q = diag(Q_diag);
R = diag(R_diag);

x1 = phi(end,1);
x2 = eps(end,1);
x3 = lamb(end,1);
x4 = phi(end,2);
x5 = eps(end,2);
x6 = lamb(end,2);
u1 = vf(end,1);
u2 = vb(end,1);

A = compute_A(x1, x2, x3, x4, x5, x6, u1, u2);
B = compute_B(x1, x2, x3, x4, x5, x6, u1, u2);

[K, ~, ~] = lqr(A, B, Q, R, zeros(6, 2));