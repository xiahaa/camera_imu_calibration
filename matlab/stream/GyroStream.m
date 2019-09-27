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
            if size(obj.data,1) ~= 3
                obj.data = obj.data';
            end
        end
        function num = num_samples(obj)
            num = size(obj.data,2);
        end
        function obj=prefilter(obj,do_plot)
            if do_plot == true
            h = getfigure('gyro sample',[400,400],'center');
            subplot(h.ax);
            subplot(3,1,1);title('gyro-x');hold on;grid on;
            plot(obj.data(1,:));
            subplot(3,1,2);title('gyro-y');hold on;grid on;
            plot(obj.data(2,:));
            subplot(3,1,3);title('gyro-z');hold on;grid on;
            plot(obj.data(3,:));
            axis tight;
            end
            obj.data = post_process_L3G4200D_data(obj.data,0.5,0.9);
            if do_plot == true
            subplot(3,1,1);title('gyro-x');legend('raw','filter');
            plot(obj.data(1,:));
            subplot(3,1,2);title('gyro-y');legend('raw','filter');
            plot(obj.data(2,:));
            subplot(3,1,3);title('gyro-z');legend('raw','filter');
            plot(obj.data(3,:));
            drawnow;
            waitforbuttonpress;
            close(h.fig);
            end
        end
        function q = integrate(obj,dt)
            if isempty(obj.last_dt) || dt ~= obj.last_dt
                obj.last_q = integrate_gyro_quaternion_uniform(obj.data, dt, []);
                obj.last_dt = dt;
            end
            q = obj.last_q;
        end
    end
end
    
