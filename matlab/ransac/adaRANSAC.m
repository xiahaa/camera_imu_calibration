function [M,final_consensus] = adaRANSAC(model_func, eval_func, data, num_points, threshold, recalculate)
    num_iter = 1e6;
    logpd = log(1 - 0.9999);
    k = 0;

    M = [];
    max_consensus = 0;
%     all_idx = 1:size(data,2);
    final_consensus = [];
    
    while k < num_iter
%         all_idx = all_idx(randperm(length(all_idx)));
%         model_set = all_idx(1:num_points);
        all_idx = randperm(size(data,2));
        model_set = all_idx(1:num_points);
        x = data(:, model_set);
        m = model_func(x);
        model_error = eval_func(m, data);
        consensus_idx = find(model_error < threshold);
        if length(consensus_idx) > max_consensus
            M = m;
            max_consensus = length(consensus_idx);
            final_consensus = consensus_idx;
            % update maxiteration number
            p_inleir = max_consensus / length(model_error);
            new_est_max_iters = round(logpd / log(1-p_inleir^num_points));
            num_iter = new_est_max_iters;
        end
        k = k+1;
    end
    if recalculate
        final_consensus_set = data(:, final_consensus);
        M = model_func(final_consensus_set);
    end
end