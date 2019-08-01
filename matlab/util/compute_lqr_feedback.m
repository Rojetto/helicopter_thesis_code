n = 6;

if n == 8
    Q_diag = [2, 10, 4, 0.2, 0.2, 0.1, 0.01, 0.01];
else
    Q_diag = [2, 25, 15, 1, 0.2, 20];
end
R_diag = [0.1, 0.1];


Q = diag(Q_diag);
R = diag(R_diag);

x1 = phi(end,1);
x2 = eps(end,1);
x3 = lamb(end,1);
x4 = phi(end,2);
x5 = eps(end,2);
x6 = lamb(end,2);
if n == 8
    x7 = vf(end,1);
    x8 = vb(end,1);
end
u1 = vf(end,1);
u2 = vb(end,1);

if n == 8
    A = compute_A_8_states(x1, x2, x3, x4, x5, x6, x7, x8, u1, u2);
    B = compute_B_8_states(x1, x2, x3, x4, x5, x6, x7, x8, u1, u2);
else
    A = compute_A_6_states(x1, x2, x3, x4, x5, x6, u1, u2);
    B = compute_B_6_states(x1, x2, x3, x4, x5, x6, u1, u2);
end

[K, ~, ~] = lqr(A, B, Q, R, zeros(n, 2));