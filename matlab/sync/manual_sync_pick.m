function [flow_idx, gyro_idx] = manual_sync_pick(flow, gyro_ts, gyro)
    h = figure('Name','Manual Pick', 'NumberTitle','off', 'Menubar','none', ...
            'Pointer','cross', 'Resize','off', 'Position',[200 200 400 400]);
    ax = axes('Parent',h, 'Units','normalized', 'Position',[0 0 1 1]);
    subplot(ax);
    subplot(2,1,1);
    plot(flow);hold on;grid on;
    title('Select two points');
    xs = ginput(2);
    selected_frames = [(round(xs(1,1))), (round(xs(2,1)))];
    plot(selected_frames, flow(selected_frames), 'ro');
    subplot(2,1,2);
    plot(gyro_ts, gyro);hold on;grid on;
    title('Select corresponding sequence in gyro data');
    xs = ginput(2);
    gyro_idxs(1) = find(gyro_ts >= xs(1,1),1);
    gyro_idxs(2) = find(gyro_ts >= xs(2,1),1);
    plot(gyro_ts(gyro_idxs), gyro(gyro_idxs), 'ro');grid on;
    title('Ok, click to continue to next')
    w = waitforbuttonpress;
    close(h);
    flow_idx = selected_frames;
    gyro_idx = gyro_idxs;
end