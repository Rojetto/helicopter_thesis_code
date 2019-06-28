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

phi = filtfilt(b, a, log.signals(3).values(i));
dphi = numdiff(phi, h, 1, 2);
ddphi = numdiff(phi, h, 2, 2);

eps = filtfilt(b, a, log.signals(4).values(i));
deps = numdiff(eps, h, 1, 2);
ddeps = numdiff(eps, h, 2, 2);

lamb = filtfilt(b, a, log.signals(5).values(i));
dlamb = numdiff(lamb, h, 1, 2);
ddlamb = numdiff(lamb, h, 2, 2);

uf = max(min(log.signals(1).values(i), 5), -5);
ub = max(min(log.signals(2).values(i), 5), -5);
end

