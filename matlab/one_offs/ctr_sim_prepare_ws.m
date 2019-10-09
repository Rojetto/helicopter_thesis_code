logts = log.time;
n_signals = numel(log.signals);
logvals = zeros(numel(logts), n_signals);

for i=1:n_signals
    logvals(:, i) = log.signals(i).values;
end

clear i n_signals