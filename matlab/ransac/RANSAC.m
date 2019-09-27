function [M,final_consensus] = RANSAC(model_func,eval_func, data, num_points, num_iter, threshold, recalculate)
% This RANSAC implementation will choose the best model based on the number of points in the consensus set. 
% At evaluation time the model is created using num_points points. Then it will be recalculated using the points in the consensus set.
    M = [];
    max_consensus = 0;
    all_idx = 1:size(data,2);
    final_consensus = [];
    for k = 1:num_iter
        all_idx = all_idx(randperm(length(all_idx)));
        model_set = all_idx(1:num_points);
        x = data(:, model_set);
        try
            m = model_func(x);
        catch
            error('');
        end
        model_error = eval_func(m, data);
        consensus_idx = find(model_error < threshold);
        if length(consensus_idx) > max_consensus
            M = m;
            max_consensus = length(consensus_idx);
            final_consensus = consensus_idx;
        end
    end
    if recalculate
        final_consensus_set = data(:, final_consensus);
        M = model_func(final_consensus_set);
    end
end

         