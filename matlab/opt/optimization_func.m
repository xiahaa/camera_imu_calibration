function robust_errors = optimization_func(x, slices, slice_sample_idxs, camera, gyro, norm_c)
    % unpack variables
    Fg = x(1);
    offset = x(2);
    gyro_bias = x(3:5);
    rot_x = x(6);
    rot_y = x(7);
    rot_z = x(8);
    % rotation matrix
    v = [rot_x;rot_y;rot_z];
    theta = norm(v);
    v = v ./ theta;
    R_g2c = axis_angle_to_rotation_matrix(v,theta);
    % gyro dt using updated rate
    Tg = (1.0/Fg);
    % dt per line scan
    row_delta = camera.readout / camera.rows;
    % final errors
    errors = [];
    % margin of integration is amount of gyro samples per frame
    integration_margin = round(ceil(Fg*camera.readout));
    
    for i = 1:length(slices)
        if length(slice_sample_idxs{i}) < 1
            continue;
        end
        slice = slices{i};
        % synchronized time for the first frame in the slice
        t_start = slice.s / camera.frame_rate + offset;
        % synchonized time for the last frame in the slice
        t_end = slice.e / camera.frame_rate + offset;
        % index for synchronized gyro to slice start
        [slice_start,~] = sample_at_time(t_start, Fg);
        % index for synchronized gyro to slice end
        [slice_end,~] = sample_at_time(t_end, Fg) ;
        %
        slice_end=slice_end+1;
        
        % gyro samples to integrate within
        integration_start = slice_start;
        % leave a margin since the initial offset is based on GS property
        integration_end = slice_end + integration_margin;
        
        % handle extreme cases
        if integration_start < 0 || integration_end >= gyro.num_samples
            % just skip
            continue
        else
            gyro_part=gyro.data(:,integration_start+1:integration_end+1);
        end
        
        % remove bias
        gyro_part_unbiased = gyro_part + gyro_bias;
        % use updated integration time
        q = integrate_gyro_quaternion_uniform(gyro_part_unbiased,Tg,[]);
        
        slice_sample = slice_sample_idxs{i};
        for j = 1:length(slice_sample)
            track = cellfun(@(t)(t(slice_sample(j))),slice.points);
%             slice.points{j};
            x = track{1}';
            y = track{end}';

            % get row time
            tx = t_start + x(2) * row_delta;
            ty = t_end + y(2) * row_delta;
            
            
            % sample index and interpolation value for point correspondence
            [nx, taux] = sample_at_time(tx, Fg);% nearest but lower
            [ny, tauy] = sample_at_time(ty, Fg);

            % interpolation using slerp
            a = nx - integration_start; % index relative to the buffer
            b = ny - integration_start;
            
            a = max(1,a);
            b = min(b,size(q,2));
            
            try
                qx = slerp(q(:,a+1),q(:,a+2),taux);
                qy = slerp(q(:,b+1),q(:,b+2),tauy);
            catch
                error('ss');
            end
            
            % to rotation matrix
            Rx = quat_to_rotation_matrix(qx);
            Ry = quat_to_rotation_matrix(qy);
            dR = Rx'*Ry;
            % TODO: what is the definition of the rotation exactly
            R = R_g2c * dR * R_g2c';
            Y = camera.unproject(y);
            Xhat = R*Y;
            xhat = camera.project(Xhat);
            % compute err and append to errrors
            err = x - xhat;
            errors = [errors;err];
            % symmetric errors
            dR1 = Ry'* Rx;
            R1 = R_g2c * (dR1) * (R_g2c');
            X = camera.unproject(x);
            Yhat = R1*X;
            yhat = camera.project(Yhat);

            err = y - yhat;
            errors = [errors;err];
        end
    end
    % apply robust norm
    robust_errors = robust_norm(errors,norm_c);
end

% compute sample index and dt to nearest integer index
function [n,tau] = sample_at_time(t,rate)
    s = t*rate - 0.5;
    n = (floor(s));
    tau = s - n;
end

% robust kernel for suppress the influence of outliers
function rn = robust_norm(r,c)
    rn = r ./ (1+(abs(r)/c));
end