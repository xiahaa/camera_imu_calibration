function data_filtered = post_process_L3G4200D_data(data, varargin)
    [b,a] = iirnotch(0.8, 0.03);
    
    data_filtered = data;
    
    for i = 1:size(data,1)
        data_filtered(i,:) = filtfilt(b,a,data(i,:));
    end
   
end