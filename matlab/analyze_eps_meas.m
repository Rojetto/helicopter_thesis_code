log = load('elevation_steps.mat');
log = log.log;

t = log.time;
vf = log.signals(1).values;
vb = log.signals(2).values;
eps = log.signals(4).values;

close all
figure
plot(t, eps)
hold on
plot(t, vf * 0.1 - 0.5)
ylim([-0.6, 0.6])

start_times = (11.5:3:161.5);
end_times = start_times + 1.2;
% sections = [7.45 9.48;
%             12.8 14.54;
%             17.47 19.8;
%             22.5 24.6;
%             27.8 29.55;
%             32.6 34.68;
%             53.42 54.91;
%             57.52 59.33;
%             62.68 64.41;
%             67.82 69.79;
%             72.68 74.54;
%             77.8 79.72;
%             84.15 84.86];
sections = [start_times', end_times'];

sections_size = size(sections);
n_datapoints = sections_size(1);

avg_eps = zeros(n_datapoints, 1);        
avg_vf = zeros(n_datapoints, 1);
avg_vb = zeros(n_datapoints, 1);

for i=1:n_datapoints
    indices = t > sections(i,1) & t < sections(i,2);
    avg_eps(i) = mean(eps(indices));
    avg_vf(i) = mean(vf(indices));
    avg_vb(i) = mean(vb(indices));
end

avg_vs = avg_vf + avg_vb;

Fs_norm = cos(avg_eps);

figure
plot(avg_vs, Fs_norm)
hold on
plot(avg_vs, Fs_norm, 'rx')
xlabel('Vs')
ylabel('cos(eps)')

figure
plot(avg_vs, rad2deg(avg_eps), 'rx')
xlabel('Vs')
ylabel('eps (deg)')

figure
plot(avg_vs, sin(avg_eps), 'rx')
xlabel('Vs')
ylabel('sin(eps)')