classdef GyroStream < handle
    properties
        last_dt;
        last_q;
        data;
    end
    methods
        function obj = GyroStream()
            obj.last_dt = [];
            obj.last_q = [];
            obj.data = [];
        end
        function from_csv(obj,filename)
            obj.data = csvread(filename);
            if size(obj.data) ~= 3
                obj.data = obj.data';
            end
        end
        function num = num_samples(obj)
            num = size(obj.data,2);
        end
        function obj=prefilter(obj)
            obj.data = post_process_L3G4200D_data(obj.data);
        end
        function q = integrate(obj,dt)
            if dt ~= obj.last_dt
                obj.last_q = integrate_gyro_quaternion_uniform(obj.data, dt);
                obj.last_dt = dt;
            end
            q = obj.last_q;
        end
    end
end
    
