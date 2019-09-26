function R=to_rot_matrix(r)
    theta = norm(r);
    if theta < 1e-10
        R = eye(3);return;
    end
    axis = r./theta;
    R = axis_angle_to_rotation_matrix(axis, theta);
end