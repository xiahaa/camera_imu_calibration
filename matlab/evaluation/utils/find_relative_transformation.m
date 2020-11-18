function [g_pos1_marker_refine,final_consensus1] = find_relative_transformation(marker_pos_body, bird_pos_body)
    % it is not a pure rigid transformation, but some t will be coupled
    % with R, so I will use an optimization to find out those parameters
    % that will optimally align them
    
    
    
    
    % firstly, call RANSAC rigid transformation to get the initial value
    [Ropt1,topt1,final_consensus1] = estimate_rigid_body_transformation_RANSAC(marker_pos_body, bird_pos_body, 1);
        
%     inlier_marker_pos_body = marker_pos_body(:, final_consensus1);
%     inlier_bird_pos_body = bird_pos_body(:, final_consensus1);
%     
%     inlier_marker_pos_body = Ropt1 * (inlier_marker_pos_body) + topt1;
    l = vecnorm(bird_pos_body - marker_pos_body);
%     figure
% %     plot(l);
%     subplot(3,1,1)
%     hold off
%     plot(marker_pos_body(1,:),'b');hold on;
%     plot(bird_pos_body(1,:),'r');hold on;
%     subplot(3,1,2)
%     plot(marker_pos_body(2,:),'b');hold on;
%     plot(bird_pos_body(2,:),'r');hold on;
%     subplot(3,1,3)
%     plot(marker_pos_body(3,:),'b');hold on;
%     plot(bird_pos_body(3,:),'r');hold on;
    
    x0 = [logSO3(Ropt1);topt1;zeros(3,1)];
    options = optimset('Display','off','TolFun',1e-10,'TolX',1e-10,'MaxIter',1000,'MaxFunEvals',10000,'Algorithm','levenberg-marquardt');
    x = lsqnonlin(@cost_func_total,x0,[],[],options,inlier_marker_pos_body,inlier_bird_pos_body);
    
    R = expSO3(x(1:3));
    t = x(4:6);
    that = x(7:9);
    
    % final
    g_pos1_marker_refine = R * (marker_pos_body + that) + t;
end

function cost = cost_func_total(x,ptsrc,ptdst)
    R = expSO3(x(1:3));
    t = x(4:6);
    that = x(7:9);% this part is due to the uncertainty of lever arm calibration
    
    pred_ptdst = R * (ptsrc + that) + t;
    err = pred_ptdst - ptdst;
    
    cost = sum(vecnorm(err).^2);
end