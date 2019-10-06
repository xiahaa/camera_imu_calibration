function q = slerp(q1, q2, u)
%     """SLERP: Spherical linear interpolation between two unit quaternions.
% 
%     Parameters
%     ------------
%     q1 : (4, 1) 
%             Unit quaternion (first element scalar)
%     q2 : (4, 1) 
%             Unit quaternion (first element scalar)
%     u : float
%             Interpolation factor in range [0,1] where 0 is first quaternion
%             and 1 is second quaternion.
% 
%     Returns
%     -----------
%     q : (4, 1) 
%             The interpolated unit quaternion
%     """
    costheta = dot(q1, q2);
    if abs(u-0)<1e-6
        q = q1;
    elseif abs(u-1.)<1e-6
        q = q2;
    elseif u > 1 || u < 0
        error("u must be in range [0, 1]");
    else
        if costheta < 0
            costheta = -costheta;
            q2 = -q2;
        end
        if abs(costheta-1)<1e-6
            q=q1;
        else
            theta = acos(costheta);
            f1 = sin((1.0 - u)*theta) / sin(theta);
            f2 = sin(u*theta) / sin(theta);
            q = f1*q1 + f2*q2;
            q = q / norm(q);
        end
    end
end
