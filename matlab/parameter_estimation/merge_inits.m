function merged_inits = merge_inits( first_init, varargin )
    merged_inits = first_init;
    
    for i=1:numel(varargin)
        init_to_merge = varargin{i};
        
        for j=1:numel(merged_inits)
            merged_inits(j).Value = [merged_inits(j).Value, init_to_merge(j).Value];
            merged_inits(j).Minimum = [merged_inits(j).Minimum, init_to_merge(j).Minimum];
            merged_inits(j).Maximum = [merged_inits(j).Maximum, init_to_merge(j).Maximum];
            merged_inits(j).Fixed = [merged_inits(j).Fixed, init_to_merge(j).Fixed];
        end
    end
end

