function data_filtered = post_process_L3G4200D_data(data, varargin)
    if nargin == 3
        wn = varargin{1};
        bandwidth = varargin{2};
    else
        wn = 0.8;
        bandwidth = 0.03;
    end
    [b,a] = iirnotch(wn, bandwidth);
    
    data_filtered = data;
    
    for i = 1:size(data,1)
        data_filtered(i,:) = filtfilt(b,a,data(i,:));
    end
   
end