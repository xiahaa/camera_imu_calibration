function [t, bird_pos_in_body, body_pos_g, Rs] = estimate_bird_pos(imugps, bird_imugps,gtime)
    t = zeros(1, length(gtime));
    bird_pos_in_body = zeros(3, length(gtime));
    for i = 1:length(gtime)
        [minval1,minid1] = min(abs(bird_imugps.GPSTime-gtime(i)));
        [minval2,minid2] = min(abs(imugps.GPSTime-gtime(i)));
        if (minval1 - minval2) < 0.008
            rpy = [imugps.yaw(minid2);imugps.pitch(minid2);imugps.roll(minid2)];
            R = angle2dcm(rpy(1),rpy(2),rpy(3));
            t(i) = imugps.GPSTime(minid2);
            bird_pos_in_body(:,i) = R'*(bird_imugps.p(:,minid1) - imugps.p(:,minid2)); 
            body_pos_g(:,i) = imugps.p(:,minid2);
            Rs(:,:,i) = R;
        end
    end
end