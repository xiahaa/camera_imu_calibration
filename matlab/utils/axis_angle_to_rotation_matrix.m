function R = axis_angle_to_rotation_matrix(v,theta)
    if abs(theta) < 1e-10
        R = eye(3);
        return;
    end
    if size(v,1)~=3
        v=v';
    end
    vhat = hat(v);
    vvt = v*v';
    R = eye(3)*cos(theta) + (1-cos(theta))*vvt + vhat * sin(theta);
end