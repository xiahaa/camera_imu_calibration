function [R, t, dist, ransac_consensus_idx] = estimate_rotation_procrustes_ransac(x, y, camera, threshold, inlier_ratio, refine)
% Calculate rotation between two sets of image coordinates using ransac.
% Inlier criteria is the reprojection error of y into image 1.
% Parameters
%     -------------------------
%     x : array 2xN image coordinates in image 1
%     y : array 2xN image coordinates in image 2
%     camera : Camera model
%     threshold : float pixel distance threshold to accept as inlier
%     do_translation : bool Try to estimate the translation as well
%     
% Returns
%     ------------------------
%     R : array 3x3 The rotation that best fulfills X = RY
%     t : array 3x1 translation if do_translation is False
%     residual : array pixel distances ||x - xhat|| where xhat ~ KRY (and lens distorsion)
%     inliers : array Indices of the points (in X and Y) that are RANSAC inliers

    X = camera.unproject(x);
    Y = camera.unproject(y);
    
    if isempty(inlier_ratio)
        inlier_ratio = 0.5;
    end
    
    function err = eval(model, data)
        Y1 = data(4:6,:);
        x1 = data(7:8,:);
        R1 = model{1};
        if length(model) == 2
            t1 = model{2};
        else
            t1 = zeros(3,1);
        end

        Xhat = R1*Y1 + t1 * ones(1,size(data,2));
        xhat = camera.project(Xhat);
        err = sqrt(sum((x1-xhat).^2,1));
    end
    
    data = [X; Y; x];
    model_func = @fun;
    eval_func = @eval;
    
    inlier_selection_prob = 0.99999;
    model_points = 2;
    ransac_iterations = int32(log(1 - inlier_selection_prob) / log(1-inlier_ratio^model_points));
    
    [model_est, ransac_consensus_idx] = RANSAC(model_func, eval_func, data, model_points, ransac_iterations, threshold, refine);
    
    if ~isempty(model_est)
        R=model_est{1};
        if length(model_est) == 2
            t = model_est{2};
        else
            t = [];
        end
        dist = eval_func(model_est, data);
    else
        dist = [];
        R=[];
        t = [];
        ransac_consensus_idx = [];
    end
end

function model = fun(data)
    [R,t] = procrustes(data(1:3,:), data(4:6,:), false);
    if isempty(t)
        model{1} = R;
    else
        model{1}=R;
        model{2}=t;
    end
end



    
