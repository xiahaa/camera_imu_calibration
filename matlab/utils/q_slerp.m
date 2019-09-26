function qs = q_slerp(t, timestamps, qlist)
    %Get the gyro rotation at time t using SLERP.
    [~,id]=min(abs(timestamps-t));
    if timestamps(id) < t
        t0 = timestamps(id);
        if id+1<length(qlist)
            t1 = timestamps(id+1);
        else
            qs = qlist(:,end);
            return;
        end
    else
        if id-1>0
            t0 = timestamps(id-1);
            t1 = timestamps(id);
        else
            qs = qlist(:,1);
            return;
        end
    end
    tau = (t - t0) / (t1 - t0);
    q1 = qlist(:, idx - 1);
    q2 = qlist(:, idx);
    qs = slerp(q1, q2, tau);
end










    