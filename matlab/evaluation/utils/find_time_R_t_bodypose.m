% R1: from the calibration programme is the rotation from imu1 to camera
% R2: from the calibration programme is the rotation from imu2 to imu1
% t2: from the calibration programme is the translation from imu2 to imu1
function [gtime, Rs, ts, marker_pos_in_imu] = find_time_R_t_bodypose(imugps, time, pos, timeoffset, R1, R2, t2,varargin)
    marker_pos_in_camera = pos(:,2:4)';%3xN
    % ideally, there should be a t1 
    marker_pos_in_imu = R2' * (R1' * marker_pos_in_camera ) - t2;
    Rs = zeros(3,3,size(marker_pos_in_imu,2));
    ts = zeros(3,1,size(marker_pos_in_imu,2));
    gtime = zeros(1, length(time));
    
    time_in_second = 0;
    if ~isempty(varargin)
        time_in_second = varargin{1};
    end
    if time_in_second == 0
        time(:,2) = time(:,2)/1e3;
    end
    
    for i = 1:length(time)
        t = time(i,2) + timeoffset;
        [minval,minid] = min(abs(imugps.time-(t)));
        if minval < 0.01
            uavpos = imugps.p(:,minid);%3x1
            rpy = [imugps.yaw(minid);imugps.pitch(minid);imugps.roll(minid)];
            R = angle2dcm(rpy(1),rpy(2),rpy(3));
            Rs(:,:,i) = R;
            ts(:,:,i) = uavpos;
            gtime(i) = imugps.GPSTime(minid);
        else
            warning('data missing');
        end
    end
end