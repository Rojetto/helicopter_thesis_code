function [ t, phi, dphi, ddphi, eps, deps, ddeps, lamb, dlamb, ddlamb, uf, ub ] = load_and_diff(file_name, t_start, t_end)
log = load(file_name);
log = log.log;

if ~exist('t_start', 'var')
    t_start = log.time(1);
end

if ~exist('t_end', 'var')
    t_end = log.time(end);
end

i = log.time >= t_start & log.time <= t_end;

t = log.time(i);

h = 0.002;
cutoff = 1.5;
[b, a] = butter(5, 2*cutoff*h);

phi = log.signals(3).values(i);
phi_filt = filtfilt(b, a, phi);
dphi = numdiff(phi_filt, h, 1, 2);
ddphi = numdiff(phi_filt, h, 2, 2);

eps = log.signals(4).values(i);
eps_filt = filtfilt(b, a, eps);
deps = numdiff(eps_filt, h, 1, 2);
ddeps = numdiff(eps_filt, h, 2, 2);

lamb = log.signals(5).values(i);
lamb_filt = filtfilt(b, a, lamb);
dlamb = numdiff(lamb_filt, h, 1, 2);
ddlamb = numdiff(lamb_filt, h, 2, 2);

uf = max(min(log.signals(1).values(i), 5), -5);
ub = max(min(log.signals(2).values(i), 5), -5);
end

