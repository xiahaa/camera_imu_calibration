classdef GPS_IMU_Stream < handle
    properties
        GPSTime;     % gps timestamp
        p;          % local px, py, pz
        vb;         % velocity body x,y,z
        ab;         % acceleration body x,y,z
        roll;        % roll angle
        pitch;       % pitch
        yaw;         % yaw
        w;          % angular velocity x,y,z
        acc_bias;  
        gyro_bias; 
        dt;
        vg;
        ag;
        time;
    end
    methods
        function obj=GPS_IMU_Stream()
            obj.GPSTime = [];     
            obj.p = []; 
            obj.vb = [];     
            obj.ab = [];    
            obj.roll = [];         
            obj.pitch = [];  
            obj.yaw = []; 
            obj.w = [];   
            obj.acc_bias = [];   
            obj.gyro_bias = []; 
            
            obj.dt = []; 
            obj.vg = []; 
            obj.ag = []; 
            obj.time = [];
        end
        
        function from_mat(obj,filename,varargin)
            imudata = load(filename);
            imudata = imudata.imugps;
            % convert to local time
            localtime = imudata(:,1) - imudata(1,1);
            if nargin >= 3
                start_time = varargin{1};
            else
                start_time = 0;
            end        
            valids = localtime >= start_time;
            obj.time = localtime(valids)';
            obj.GPSTime = imudata(valids,1)';
            obj.p = imudata(valids,2:4)';
            obj.vb = imudata(valids,5:7)';
            obj.ab = imudata(valids,8:10)';
            obj.roll = imudata(valids,11)';
            obj.pitch = imudata(valids,12)';
            obj.yaw = imudata(valids,13)';
            obj.w = imudata(valids,14:16)';
            obj.acc_bias = imudata(valids,17:19)';
            obj.gyro_bias = imudata(valids,20:22)';
            
            % compute dt
            dts = obj.GPSTime(2:end) - obj.GPSTime(1:end-1);
            obj.dt = mean(dts);
            
            % from angle to rad
            obj.roll = obj.roll * pi / 180.0;
            obj.pitch = obj.pitch * pi / 180.0;
            obj.yaw = obj.yaw * pi / 180.0;
            
            obj.w = obj.w * pi / 180.0;
            obj.gyro_bias = obj.gyro_bias * pi / 180.0;
            
            obj.get_vg_ag();
            
            obj.w = obj.notch_filter(obj.w);

            disp(['num of samples:',num2str(size(obj.GPSTime,2))]);
        end
        
        function from_csv(obj,filename,varargin)
            imudata = uiimport(filename);
            imudata = imudata.data;
            % convert to local time
            localtime = imudata(:,1) - imudata(1,1);
            if nargin >= 3
                start_time = varargin{1};
            else
                start_time = 0;
            end        
            valids = localtime >= start_time;
            obj.time = localtime(valids)';
            obj.GPSTime = imudata(valids,1)';
            obj.p = imudata(valids,2:4)';
            obj.vb = imudata(valids,5:7)';
            obj.ab = imudata(valids,8:10)';
            obj.roll = imudata(valids,11)';
            obj.pitch = imudata(valids,12)';
            obj.yaw = imudata(valids,13)';
            obj.w = imudata(valids,14:16)';
            obj.acc_bias = imudata(valids,17:19)';
            obj.gyro_bias = imudata(valids,20:22)';
            
            % compute dt
            dts = obj.GPSTime(2:end) - obj.GPSTime(1:end-1);
            obj.dt = mean(dts);
            
            % from angle to rad
            obj.roll = obj.roll * pi / 180.0;
            obj.pitch = obj.pitch * pi / 180.0;
            obj.yaw = obj.yaw * pi / 180.0;
            
            obj.w = obj.w * pi / 180.0;
            obj.gyro_bias = obj.gyro_bias * pi / 180.0;
            
            obj.get_vg_ag();
            
            obj.w = obj.notch_filter(obj.w);

            disp(['num of samples:',num2str(size(obj.GPSTime,2))]);
        end
        function num = num_samples(obj)
        	num = size(obj.GPSTime,2);
        end
        function get_vg_ag(obj)
            obj.vg = obj.vb;
            obj.ag = obj.ab;
            for i = 1:size(obj.time,2)
                R = rpytoR([obj.yaw(i),obj.pitch(i),obj.roll(i)]);
                obj.vg(:,i) = R*obj.vb(:,i);
                obj.ag(:,i) = R*obj.ab(:,i);
            end
        end
    end
    methods(Static)
        function data_filtered = notch_filter(data,varargin)
            if nargin == 3
                Wn = varargin{1};
                bandwidth = varargin{2};
            elseif nargin ~= 1
                ME = MException('GPS_IMU_Stream:notch_filter', ...
                    'Wrong Inputs');
                throw(ME);
            else
                Wn = 0.5;
                bandwidth = 0.5;
            end
            
            [b,a] = iirnotch(Wn, bandwidth);
    
            data_filtered = data;
    
            for i = 1:size(data,1)
                data_filtered(i,:) = filtfilt(b,a,data(i,:));
            end
        end
    end
end

