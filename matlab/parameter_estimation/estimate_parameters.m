p1 = 0.3117;
q1 = 0.9247/2;
p2 = 0.1396;
q2 = 0.7637/2;

lp = 0.178;
lh = 0.67;

h = 0.002;

load sine_travel

t = log.time;
Ff = min(Fr(log.signals(1).values, p1, q1, p2, q2), 5);
Fb = min(Fr(log.signals(2).values, p1, q1, p2, q2), 5);
phi = log.signals(3).values;
eps = log.signals(4).values;

cutoff = 1.5;
[b, a] = butter(5, 2*cutoff*h);
%freqz(b, a)
phi_filter = filtfilt(b, a, phi);
eps_filter = filtfilt(b, a, eps);

dphi = numdiff(phi_filter, h, 1, 2);
ddphi = numdiff(phi_filter, h, 2, 2);

deps = numdiff(eps_filter, h, 1, 2);
ddeps = numdiff(eps_filter, h, 2, 2);


%sample_shift = 52;
close all

figure('Name', 'phi and numerical derivatives')
hold on
plot(t, phi)
plot(t, phi_filter)
plot(t, dphi)
plot(t, ddphi)
%plot(t(3:end-2-sample_shift), dphi_int(1+sample_shift:end))
%plot(t(3:end-2-sample_shift), phi_int(1+sample_shift:end))

figure('Name', 'eps and numerical derivatives');
hold on
plot(t, eps)
plot(t, eps_filter)
plot(t, deps)
plot(t, ddeps)


n = numel(t);

A = zeros(2*n, 6);
b = zeros(2*n, 1);

A(1:2:end, 1) = ddphi;
A(1:2:end, 2) = dphi;
A(2:2:end, 3) = ddeps;
A(2:2:end, 4) = deps;
A(2:2:end, 5) = sin(eps);
A(2:2:end, 6) = cos(eps);

b(1:2:end) = lp.*(Ff-Fb);
b(2:2:end) = lh.*cos(phi).*(Ff+Fb);

%p = (A'*A) \ (A'*b)
p = lsqlin(A, b, [0 -1 0 0 0 0; 0 0 0 -1 0 0], [0; 0])