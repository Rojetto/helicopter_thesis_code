Q_diag = [2, 100, 150, 1, 0.2, 20];
R_diag = [1, 1];

tune_for_end = false;
%%


Q = diag(Q_diag);
R = diag(R_diag);

if tune_for_end
    x1 = phi(end,1);
    x2 = eps(end,1);
    x3 = lamb(end,1);
    x4 = phi(end,2);
    x5 = eps(end,2);
    x6 = lamb(end,2);

    u1 = vf(end,1);
    u2 = vb(end,1);
else
    x1 = 0; x2 = 0; x3 = 0; x4 = 0; x5 = 0; x6 = 0;
    u1 = 3.51; u2 = 3.51;
end

A = compute_A_6_states(x1, x2, x3, x4, x5, x6, u1, u2);
B = compute_B_6_states(x1, x2, x3, x4, x5, x6, u1, u2);

[K, ~, ~] = lqr(A, B, Q, R, zeros(6, 2));