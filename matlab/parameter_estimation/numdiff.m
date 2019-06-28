function diff_data_ext = numdiff(data, h, order, accuracy)
    if order == 1
        if accuracy == 1
            numerator = -1/2*data(1:end-2) + 1/2*data(3:end);
        elseif accuracy == 2
            numerator = 1/12*data(1:end-4) - 2/3*data(2:end-3) + 2/3*data(4:end-1) - 1/12*data(5:end);
        end
        
        diff_data = numerator / h;
    elseif order == 2
        if accuracy == 1
            numerator = 1*data(1:end-2) - 2*data(2:end-1) + 1*data(3:end);
        elseif accuracy == 2
            numerator = -1/12*data(1:end-4) + 4/3*data(2:end-3) - 5/2*data(3:end-2) + 4/3*data(4:end-1) - 1/12*data(5:end);
        end
        
        diff_data = numerator / h^2;
    end
    
    half_size_diff = (numel(data) - numel(diff_data))/2;
    
    diff_data_ext = zeros(numel(data), 1);
    diff_data_ext(1 + half_size_diff:end-half_size_diff) = diff_data;
    
    for i=1:half_size_diff
        diff_data_ext(i) = diff_data(1);
        diff_data_ext(end-i+1) = diff_data(end);
    end
end