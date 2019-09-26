function [axis, angle] = rotation_matrix_to_axis_angle(R)
    [evec,eval]=eig(R);
    [~,k] = min(abs(diag(eval)-1));
    axis = evec(:,k);
    vhat = [R(3,2)-R(2,3),R(1,3)-R(3,1),R(2,1)-R(1,2)]';
    sintheta = 0.5 * dot(axis,vhat);
    costheta = 0.5 * (trace(R)-1);
    angle = arctan2(sintheta,costheta);
end
