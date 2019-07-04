function [clips, iddata_obj, initial_states] = make_experiment_clips( file_names, output_variables, varargin )
%% input parsing
n_files = numel(file_names);

p = inputParser;
addRequired(p, 'file_names')
addRequired(p, 'output_variables', @validate_variables)
addOptional(p, 'InputVariables', {'uf', 'ub'}, @validate_variables)
addOptional(p, 'ClipLength', 10)
addOptional(p, 'StartTimes', zeros(n_files, 1))
addOptional(p, 'ResamplingDecimation', 20)


parse(p, file_names, output_variables, varargin{:})

output_variables = normalize_variables(output_variables);
input_variables = normalize_variables(p.Results.InputVariables);
clip_length = p.Results.ClipLength;
start_times = p.Results.StartTimes;
resampling_decimation = p.Results.ResamplingDecimation;

%% clip splitting
clip_size_samples = clip_length * 500;
clips = struct('t', {},...
               'phi', {},...
               'dphi', {},...
               'eps', {},...
               'deps', {},...
               'lamb', {},...
               'dlamb', {},...
               'uf', {},...
               'ub', {},...
               'file', {},...
               'tStart', {},...
               'tEnd', {});

for i=1:n_files
    file_name = file_names{i};
    [t, phi, dphi, ~, eps, deps, ~, lamb, dlamb, ~, uf, ub] = load_and_diff(file_name);
    
    i_start = start_times(i)*500+1;
    
    while i_start < numel(t)
        i_end = min(i_start + clip_size_samples - 1, numel(t));
        
        samples_clip = i_end - i_start + 1;
        
        clips(end+1) = struct('t', (0:(samples_clip-1))*0.002,...
                              'phi', phi(i_start:i_end),...
                              'dphi', dphi(i_start:i_end),...
                              'eps', eps(i_start:i_end),...
                              'deps', deps(i_start:i_end),...
                              'lamb', lamb(i_start:i_end),...
                              'dlamb', dlamb(i_start:i_end),...
                              'uf', uf(i_start:i_end),...
                              'ub', ub(i_start:i_end),...
                              'file', file_name,...
                              'tStart', (i_start-1)*0.002,...
                              'tEnd', i_end*0.002);
        
        i_start = i_end + 1;
    end
end

n_clips = numel(clips);

%% make iddata
output_cell = cell(n_clips, 1);
input_cell = cell(n_clips, 1);
experiment_names = cell(n_clips, 1);

for i=1:n_clips
    output_cell{i} = concatenate_array_of_variables(output_variables, clips(i));
    input_cell{i} = concatenate_array_of_variables(input_variables, clips(i));
    experiment_names{i} = [clips(i).file ': ' num2str(clips(i).tStart) ' - ' num2str(clips(i).tEnd)];
end

iddata_obj = iddata(output_cell, input_cell, 0.002, ...
    'ExperimentName', experiment_names, ...
    'InputName', input_variables, ...
    'OutputName', output_variables);
if resampling_decimation > 1
    iddata_obj = resample(iddata_obj, 1, resampling_decimation);
end

%% build initial state struct array
initial_states = struct('Name', {},...
                        'Unit', {},...
                        'Value', {},...
                        'Minimum', {},...
                        'Maximum', {},...
                        'Fixed',{});
                    
for i=1:numel(output_variables)
    output_var = output_variables{i};
    
    out_i = 2*i - 1;
    dout_i = 2*i;
    
    initial_states(out_i).Name = output_var;
    initial_states(dout_i).Name = ['d' output_var];
    initial_states(out_i).Unit = 'rad';
    initial_states(dout_i).Unit = 'rad/s';
    initial_states(out_i).Minimum = -Inf * ones(1, n_clips);
    initial_states(dout_i).Minimum = -Inf * ones(1, n_clips);
    initial_states(out_i).Maximum = Inf * ones(1, n_clips);
    initial_states(dout_i).Maximum = Inf * ones(1, n_clips);
    initial_states(out_i).Fixed = true(1, n_clips);
    initial_states(dout_i).Fixed = true(1, n_clips);
    
    if strcmp(output_var, 'phi')
        initial_states(out_i).Values = arrayfun(@(c) c.phi(1), clips);
        initial_states(dout_i).Values = arrayfun(@(c) c.dphi(1), clips);
    elseif strcmp(output_var, 'epsilon')
        initial_states(out_i).Values = arrayfun(@(c) c.eps(1), clips);
        initial_states(dout_i).Values = arrayfun(@(c) c.deps(1), clips);
    elseif strcmp(output_var, 'lambda')
        initial_states(out_i).Values = arrayfun(@(c) c.lamb(1), clips);
        initial_states(dout_i).Values = arrayfun(@(c) c.dlamb(1), clips);
    end
end
    

%% utility functions
    function out = concatenate_array_of_variables(variables, clip)
        n_variables = numel(variables);
        n_samples = numel(clip.phi);
        
        out = zeros(n_samples, n_variables);
        
        for j=1:n_variables
            if strcmp(variables{j}, 'phi')
                val = clip.phi;
            elseif strcmp(variables{j}, 'epsilon')
                val = clip.eps;
            elseif strcmp(variables{j}, 'lambda')
                val = clip.lamb;
            elseif strcmp(variables{j}, 'uf')
                val = clip.uf;
            elseif strcmp(variables{j}, 'ub')
                val = clip.ub;
            end
            
            out(:, j) = val;
        end
    end

    function normalized_value = normalize_variables(value)
        allowed_output_variables = {'phi', 'epsilon', 'lambda', 'uf', 'ub'};
        
        if ischar(value)
            value = {value};
        end
        
        normalized_value = cellfun(@(output_var) validatestring(output_var, allowed_output_variables), value, 'UniformOutput', false);
    end
    function valid = validate_variables(value)
        value = normalize_variables(value);
        
        valid = true;
    end
end

