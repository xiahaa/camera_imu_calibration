function [marker_pos_refine,final_consensus1] = find_relative_transformation_new(Rs, ts, marker_pos_body, bird_pos)
    % it is not a pure rigid transformation, but some t will be coupled
    % with R, so I will use an optimization to find out those parameters
    % that will optimally align them
    marker_pos = bird_pos;
    for i = 1:size(bird_pos,2)
        marker_pos(:,i) = Rs(:,:,i) * marker_pos_body(:,i) + ts(:,i);
    end
    l = vecnorm(bird_pos - marker_pos);
    x0 = [mean(l);zeros(3,1)];
    options = optimset('Display','off','TolFun',1e-10,'TolX',1e-10,'MaxIter',1000,'MaxFunEvals',10000,'Algorithm','levenberg-marquardt');
    x = lsqnonlin(@cost_func_total,x0,[],[],options,marker_pos,bird_pos,Rs);
    
    t = x(2:4);
    tg = marker_pos;
    for i = 1:size(marker_pos,2)
        tg(:,i) = Rs(:,:,i) * t;
    end
    
    ray = bird_pos - marker_pos - tg;
    ray = ray./vecnorm(ray);
    marker_pos_refine = marker_pos + x(1)*ray + tg;
    
    % firstly, call RANSAC rigid transformation to get the initial value
    [Ropt1,topt1,~] = estimate_rigid_body_transformation_RANSAC(marker_pos_refine, bird_pos, 1);
    
    x0 = [logSO3(Ropt1);topt1;zeros(3,1)];
    options = optimset('Display','off','TolFun',1e-10,'TolX',1e-10,'MaxIter',1000,'MaxFunEvals',10000,'Algorithm','levenberg-marquardt');
    x = lsqnonlin(@cost_func_rigid_total,x0,[],[],options,marker_pos_refine,bird_pos);
    
    R = expSO3(x(1:3));
    t = x(4:6);
    that = x(7:9);
    % final
    marker_pos_refine = R * (marker_pos_refine + that) + t;
end

function cost = cost_func_rigid_total(x,ptsrc,ptdst)
    R = expSO3(x(1:3));
    t = x(4:6);
    that = x(7:9);% this part is due to the uncertainty of lever arm calibration
    
    pred_ptdst = R * (ptsrc + that) + t;
    err = pred_ptdst - ptdst;
    
    cost = sum(vecnorm(err).^2);
end

function cost = cost_func_total(x,ptsrc,ptdst,Rs)
    l = x(1);
    t = x(2:4);

    tg = ptsrc;
    for i = 1:size(ptsrc,2)
        tg(:,i) = Rs(:,:,i) * t;
    end
    ray = ptdst - (ptsrc + tg);
    ray = ray./vecnorm(ray);
    
    ptdst_pred = ptsrc + tg + ray.*l;
    err = ptdst_pred - ptdst;
    cost = sum(vecnorm(err).^2);
end