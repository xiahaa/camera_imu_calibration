function align_pos_g = estimate_align_pos_g(Rs, body_pos_g, align_pos_body)
    align_pos_g = zeros(size(align_pos_body));
    for i = 1:size(Rs,3)
        align_pos_g(:,i) = Rs(:,:,i)*align_pos_body(:,i) + body_pos_g(:,i);
    end
end