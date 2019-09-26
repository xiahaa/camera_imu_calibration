function q_list = integrate_gyro_quaternion(gyro_ts, gyro_data)
    % Integrate angular velocities to rotations
    q_list = zeros(4,size(gyro_data,2));
    q_list(:,1) = [1;0;0;0];
    for i = 2:length(gyro_ts)
        w = gyro_data(:,i);
        dt = gyro_ts(i) - gyro_ts(i-1);
        qprev = q_list(:,i-1);
        A = [[0,    -w(1),  -w(2),  -w(3)], ...
             [w(1),  0,      w(3),  -w(2)], ...
             [w(2), -w(3),   0,      w(1)], ...
             [w(3),  w(2),  -w(1),   0]];
        qnew = (eye(4) + (dt/2.0) .* A)*(qprev);
        qnorm = norm(qnew);
        qnew = qnew./qnorm;
        q_list(:,i) = qnew;
    end
end