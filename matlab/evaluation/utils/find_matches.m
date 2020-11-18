function [bird_time, bird_pos] = find_matches(gtime,bird_imugps)
    bird_time = zeros(1, length(gtime));
    bird_pos = zeros(3, length(gtime));
    for i = 1:length(gtime)
        [minval,minid] = min(abs(bird_imugps.GPSTime-gtime(i)));
        if minval < 0.01
            bird_time(i) = bird_imugps.GPSTime(minid);
            bird_pos(:,i) = bird_imugps.p(:,minid);
        else
            warning('data missing');
        end
    end
end