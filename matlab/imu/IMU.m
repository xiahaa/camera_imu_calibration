classdef IMU < handle
    properties
        integrated;
        gyro_data;
        timestamps;
    end
    methods 
        function obj = IMU()
            obj.integrated = [];
            obj.gyro_data = [];
            obj.timestamps = [];
        end
        
        function hz = rate(obj)
            hz = mean(1./(diff(obj.timestamps)));
        end
        
        function zero_level_calibrate(obj, duration, t0)
            t1 = t0 + duration;
            id = obj.timestamps >= t0 & obj.timestamps <= t1;
            % bias
            bias = mean(obj.gyro_data(:,id));
            obj.gyro_data = obj.gyro_data - bias * ones(1,size(obj.gyro_data,2));
        end
        
        function gyro_data_corrected(obj,R)
            %Get relative pose corrected data.
            obj.gyro_data = R*obj.gyro_data;
        end
        
        function q = integrate(obj,R,uniform)
            if ~isempty(R)
                gyro_data_corrected(R);
            end
            if uniform
                q = integrate_gyro_quaternion_uniform(obj.gyro_data,obj.timestamps(2)-obj.timestamps(1),[]);
            else
                N = length(obj.timestamps);
                q = zeros(4,N);
                q0 = [1, 0, 0, 0]';% Initial rotation (no rotation)
                q(:,1) = q0;
                 % Iterate over all
                for i = 2:N
                    dt = obj.timestamps(i) - obj.timestamps(i - 1);
                    qgyro = 0.5*dt.*[0;obj.gyro_data(:,i)];
                    q(:,i) = qprod(q0, qgyro);
                    if abs(norm(q(:,i))-1) > 1e-10
                        q(:,i)=q(:,i)./norm(q(:,i));
                    end
                    q0 = q(:,i); 
                end
            end
        end
    end
end

    

    