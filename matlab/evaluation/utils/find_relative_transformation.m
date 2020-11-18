function [marker_pos_refine,final_consensus1] = find_relative_transformation(marker_pos, bird_pos)
    % it is not a pure rigid transformation, but some t will be coupled
    % with R, so I will use an optimization to find out those parameters
    % that will optimally align them
    l = vecnorm(bird_pos - marker_pos);
    x0 = mean(l);
    options = optimset('Display','off','TolFun',1e-10,'TolX',1e-10,'MaxIter',1000,'MaxFunEvals',10000,'Algorithm','levenberg-marquardt');
    x = lsqnonlin(@cost_func_total,x0,[],[],options,marker_pos,bird_pos);
    
    ray = bird_pos - marker_pos;
    ray = ray./vecnorm(ray);
    marker_pos_refine = marker_pos + x*ray;
    
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

function cost = cost_func_total(x,ptsrc,ptdst)
    ray = ptdst - ptsrc;
    ray = ray./vecnorm(ray);
    ptdst_pred = ptsrc + ray.*x;
    err = ptdst_pred - ptdst;
    cost = sum(vecnorm(err).^2);
end