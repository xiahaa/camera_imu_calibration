function [R,t,final_consensus] = estimate_rigid_body_transformation_RANSAC(ptsrc, ptdst,varargin)
    data = [ptsrc;ptdst];
    threshold = 0.2;
	if ~isempty(varargin)
		threshold = varargin{1}
	end
    recalculate = true;
    model_func = @estimate_rigid_body_transformation_SVD;
    num_points = 3;
    
    function err = eval_func(model, data)
        R = model.Ropt;
        t = model.topt;
        x = data(1:3,:);
        y = data(4:6,:);
        yhat = R*x+t;
        err = y-yhat;
        err = vecnorm(err);
    end
    
    [M,final_consensus] = adaRANSAC(model_func, @eval_func, data, num_points, threshold, recalculate);
    R = M.Ropt;
    t = M.topt;
end