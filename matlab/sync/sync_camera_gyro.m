function time_offset = sync_camera_gyro(flow, flow_timestamps, gyro_data, ...
    gyro_timestamps, varargin)
% """Get time offset that aligns image timestamps with gyro timestamps.
% 
%   Given an image sequence, and gyroscope data, with their respective timestamps,
%   calculate the offset that aligns the image data with the gyro data.
%   The timestamps must only differ by an offset, not a scale factor.
% 
%   This function finds an approximation of the offset *d* that makes this transformation
%           t_gyro = t_camera + d
% 
%   i.e. your new image timestamps should be
% 
%           image_timestamps_aligned = image_timestamps + d
% 
%   The offset is calculated using zero-mean cross correlation of the gyroscope data magnitude
%   and the optical flow magnitude, calculated from the image sequence.
%   ZNCC is performed using pyramids to make it quick.
% 
%   The offset is accurate up to about +/- 2 frames, so you should run
%   *refine_time_offset* if you need better accuracy.
% 
%   Parameters
%   ---------------
%       flow : flow strength
%       flow_timestamps : (1,N)
%       gyro_data : (3, N) Gyroscope measurements (angular velocity)
%       gyro_timestamps : (1,N) Timestamps of data in gyro_data     
%       do_plot: 
%       levels : int, Number of pyramid levels
%         
% 
%   Returns
%   --------------
%       time_offset  :  float
%             The time offset to add to image_timestamps to align the image data
%             with the gyroscope data
%     """
    if nargin >= 5
        do_plot = varargin{1};
    else
        do_plot = true;
    end
    if nargin >= 6
        levels = varargin{2};
    else
        levels = 6;
    end
    if nargin >= 7
        save_path = varargin{3};
    else
        save_path = [];
    end
    if nargin >= 8
        manual = varargin{4};
    else
        manual = false;
    end
    
    % Gyro from gyro data
    gyro_mag = vecnorm(gyro_data);
    % Resample to match highest
    rate = @(ts) (1/mean(diff(ts)));
    freq_gyro = rate(gyro_timestamps);
    freq_image = rate(flow_timestamps);

    if freq_gyro > freq_image
        rel_rate = freq_gyro / freq_image;
        flow_mag = zncc.upsample(flow, rel_rate);% could also use matlab resample
    else
        flow_mag = flow;
        rel_rate = freq_image / freq_gyro;
        gyro_mag = zncc.upsample(gyro_mag, rel_rate);
    end
    
    if ~isempty(save_path)
        save(strcat(save_path,'sig.mat'),'flow_mag','gyro_mag');
    end
    
    gyro_normalized = (gyro_mag / max(gyro_mag));
    flow_normalized = (flow_mag / max(flow_mag));
    
    % semi-autonomous
    if manual
        [flow_ids, gyro_ids] = manual_sync_pick(flow_normalized, gyro_timestamps, gyro_normalized);
        flow_mag_s = flow_mag(flow_ids(1),flow_ids(2));
        gyro_mag_s = gyro_mag(gyro_ids(1),gyro_ids(2));
    else
        flow_mag_s = flow_normalized;
        gyro_mag_s = gyro_normalized;
    end

    ishift = zncc.coarse_to_fine_corr(flow_mag_s,gyro_mag_s, 12, levels);
    % ishift = znccpyr.find_shift_pyr(flow_mag, gyro_mag, levels);
    % ishift = ishift2

    if freq_gyro > freq_image
        time_offset = -ishift * 1.0/freq_gyro;
    else
        time_offset = -ishift * 1.0/freq_image;
    end
    
    % First pick good points in flow
    if do_plot
        h = struct();
        h.fig = figure('Name','Time Alignment', 'NumberTitle','off', 'Menubar','none', ...
            'Pointer','cross', 'Resize','off', 'Position',[200 200 400 400]);
        if ~mexopencv.isOctave()
            %HACK: not implemented in Octave
            movegui(h.fig, 'west');
        end
        h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
        subplot(h.ax);
        subplot(2,1,1);
        plot(flow_timestamps, flow/max(flow), 'r-','LineWidth',2);hold on;
        plot(gyro_timestamps, gyro_mag/max(gyro_mag), 'b-','LineWidth',2);grid on;
        legend({'flow','gyro'});
        title('Before Alignment');
        subplot(2,1,2);
        plot(flow_timestamps+time_offset, flow/max(flow), 'r-','LineWidth',2);hold on;
        plot(gyro_timestamps, gyro_mag/max(gyro_mag), 'b-','LineWidth',2);grid on;
        legend({'flow','gyro'});
        title('After Alignment');
    end
end