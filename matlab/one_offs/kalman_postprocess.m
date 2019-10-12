h = 0.002;
cutoff = 1.5;
[b, a] = butter(5, 2*cutoff*h);


phi_filt = filtfilt(b, a, y_sample.signals.values(:, 1));
dphi_filt = numdiff(phi_filt, h, 1, 2);

eps_filt = filtfilt(b, a, y_sample.signals.values(:, 2));
deps_filt = numdiff(eps_filt, h, 1, 2);

lamb_filt = filtfilt(b, a, y_sample.signals.values(:, 3));
dlamb_filt = numdiff(lamb_filt, h, 1, 2);