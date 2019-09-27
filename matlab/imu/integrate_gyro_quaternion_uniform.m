function q = integrate_gyro_quaternion_uniform(gyro, dt, initial)
    N = size(gyro,2);
    q = zeros(4,N);
    dth = dt * 0.5;
    
    if isempty(initial)
        q0 = [1;0;0;0];
    else
        q0 = initial;
        if abs(norm(q0)-1) > 1e-10
            q0=q0./norm(q0);
        end
    end
    
    qgyro = dth.*[0;gyro(:,1)];
    q(:,1) = q0 + qprod(q0, qgyro);
    if abs(norm(q(:,1))-1) > 1e-10
        q(:,1)=q(:,1)./norm(q(:,1));
    end
    
    for i = 2:N
        qgyro = dth.*[0;gyro(:,i)];
        q(:,i) = q(:,i-1) + qprod(q(:,i-1), qgyro);
        if abs(norm(q(:,i))-1) > 1e-10
            q(:,i)=q(:,i)./norm(q(:,i));
        end
    end
    
end