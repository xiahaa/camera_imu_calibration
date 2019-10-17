classdef Calibration < handle
    properties
        video;
        gyro;
        slice;
        params;
        dbg_level;
        save_path;
        result;
    end
    methods
        function obj = Calibration(video,gyro,debug_level,save_path)
            obj.video = video;
            obj.gyro = gyro;
            obj.slice = [];
            obj.params = struct('users',struct(),'initialized',struct(),'calibration',struct());
            obj.dbg_level = 5;
            obj.save_path = save_path;
            obj.result = zeros(8,1);
        end
        function initialize(obj,gyro_rate,varargin)
            obj.params.users.gyro_rate = gyro_rate;
            disp(['gyro rate is ',num2str(gyro_rate)]);
            obj.params.initialized.gyro_bias = zeros(3,1);
            
            %% do time sychronization first and then extract slice
            nlevels = 6;
            time_offset = obj.find_initial_offset(nlevels);
            
            %% now generate slice
            if isempty(obj.slice)
                filename = fullfile(obj.save_path,'slice.mat'); 
                if obj.dbg_level > 0
                    if exist(filename,'file')
                        load(filename,'slice');
                        obj.slice = slice;
                    else
                        slice = Slice.from_stream_randomly(obj.video);
                        if ~exist(obj.save_path,'dir')
                            mkdir(obj.save_path);
                        end
                        save(filename,'slice');
                        obj.slice = slice;
                    end
                else
                    slice = Slice.from_stream_randomly(obj.video);
                    if ~exist(obj.save_path,'dir')
                        mkdir(obj.save_path);
                    end
                    save(filename,'slice');
                    obj.slice = slice;
                end
            end
            
            %% now estimate relative rotation 
            R = obj.find_initial_rotation();
        end
        
        function calibrate(obj, varargin)
            if nargin >= 2
                max_tracks = varargin{1};
            else
                max_tracks = 1500; 
            end
            if nargin >= 3
                max_eval = varargin{2};
            else
                max_eval = 900;
            end
            if nargin >= 4
                max_eval = varargin{3};
            else
                norm_c = 3;
            end
            
            % initialization optimization vectors

            x0 = [obj.params.users.gyro_rate;obj.params.initialized.time_offset; ...
                  obj.params.initialized.gyro_bias; ...
                  obj.params.initialized.rot_x; ...
                  obj.params.initialized.rot_y; ... 
                  obj.params.initialized.rot_z];
            
            % for each slice, use tracked points to est R and reidentify inliers
            available_tracks = 0;
            for i = 1:length(obj.slice)
                available_tracks = available_tracks + length(obj.slice{i}.inliers);
            end
            
            if available_tracks < max_tracks
                warning('ues less than assigned tracks to calibrate');
                max_tracks = available_tracks;
            end
            
            % resampling
            slice_sample_idxs = Slice.fill_sampling(obj.slice, max_tracks);
            
            % 
            f = @(x)optimization_func(x,obj.slice,slice_sample_idxs,obj.video.camera_model,obj.gyro,norm_c);
            
            %
            options = optimoptions(@lsqnonlin,'FunctionTolerance',1e-10,'MaxFunctionEvaluations',max_eval,'OptimalityTolerance',1e-10);
            
            %
            [x,~,~,flag] = lsqnonlin(f,x0,[],[],options);

            if flag == 1 || flag == 2 || flag == 3 || flag == 4
                obj.params.users.gyro_rate = x(1);
                obj.params.initialized.time_offset=x(2);
                obj.params.initialized.gyro_bias=x(3:5); 
                obj.params.initialized.rot_x=x(6); 
                obj.params.initialized.rot_y=x(7);  
                obj.params.initialized.rot_z=x(8);
            elseif flag == 0
                warning('exit due to maximum evaluation reseached, may not be a local minimum!');
            else
                error('Calibration Failed');
            end
            
            obj.result = x;
            
        end
        
        function [n0, tau] = video_time_to_gyro_sample(obj,t)
            gyro_rate = obj.params.users.gyro_rate;
            time_offset = obj.params.initialized.time_offset;
            % flow time + time offset = equivalent imu time
            % times rate to get the count
            n = gyro_rate * (t + time_offset);
            n0 = int32(floor(n));
            tau = n - n0; % for slerp interpolation
        end
        
        function R = find_initial_rotation(obj)
            dt = 1 / obj.params.users.gyro_rate;
            q = obj.gyro.integrate(dt);% integration to get the imu rotation
            
            
            len = length(obj.slice);
            video_axes = zeros(3,len);
            gyro_axes = zeros(3,len);
            final_len = 0;
            
            ransac_thereshold = 7;
            for k = 1:len
                % # estimate rotation here
                % # estimate the rotation for slice
                % # to make sure within this slice, there is
                % # a significant rotation
                s = obj.slice{k};
                s.estimate_rotation(obj.video.camera_model,ransac_thereshold);
                if isempty(s.axis)
                    continue;
                end
%                 if ~(s.angle > 0)
%                     continue;
%                 end
                
                tmp = 1 / obj.video.camera_model.frame_rate;
                
                t1 = s.s * tmp;
                [n1,~] = obj.video_time_to_gyro_sample(t1);
                t2 = s.e * tmp;
                [n2,~] = obj.video_time_to_gyro_sample(t2);
                
                try
                    qx = q(:,n1);
                    qy = q(:,n2);
                catch
                    continue;
                end
                
                % to rotation matrix
                Rx = quat_to_rotation_matrix(qx);
                Ry = quat_to_rotation_matrix(qy);
                dR = Rx'*Ry;
                [v, theta] = rotation_matrix_to_axis_angle(dR);
                if theta < 0
                    v = -v;
                end
                % add to gyro rotation axis and video rotation axis
                % here can do sth
                final_len = final_len + 1;
                gyro_axes(:,final_len)=v;
                video_axes(:,final_len)=s.axis;
                
            end
            gyro_axes(:,final_len+1:end)=[];
            video_axes(:,final_len+1:end)=[];
            
            if length(gyro_axes) < 2
                error('Hand-eye calibration requires >= 2 rotations');
            end
        
            % [0] since we only cares about rotation
            model_func = @(d) (procrustes(d(1:3,:), d(4:6,:), false));
            
            function theta=eval_func(model, d)
                X=d(1:3,:);
                Y=d(4:6,:);
                Rest = model;
                Xhat = Rest*Y;
                costheta = dot(Xhat,X);
                theta = acos(costheta);
            end 
            
            % could also try with Euclidean norm
%             function err = eval_func(model, data)
%                 X=data(1:3,:);
%                 Y=data(4:6,:);
%                 Rest = model;
%                 Xhat = Rest*Y;
%                 err = vecnorm(Xhat - X);
%             end
        
            model_points = 2; % select to not use minimal case
            threshold = 8.0*pi/180;% tunable
%             threshold = 0.2;
            
            %% only for debug
%             load('./data/rot.mat');
            
            data = [video_axes;gyro_axes];
            R = adaRANSAC(model_func, @eval_func, data, model_points, threshold, true);
            [n, theta] = rotation_matrix_to_axis_angle(R);
            so3 = theta*n;
            obj.params.initialized.rot_x = so3(1);
            obj.params.initialized.rot_y = so3(2);
            obj.params.initialized.rot_z = so3(3);
            disp([so3]);
        end
        
        function time_offset = find_initial_offset(obj,nlevels)
            filename = fullfile(obj.save_path,'flow.mat');
            if obj.dbg_level > 0
                if exist(filename,'file')
                    load(filename,'flow');
                else
                    flow = obj.video.estimate_flow(true);
                    if ~exist(obj.save_path,'dir')
                        mkdir(obj.save_path);
                    end
                    save(filename,'flow');
                end
            else
                flow = obj.video.estimate_flow(true);
                if ~exist(obj.save_path,'dir')
                    mkdir(obj.save_path);
                end
                save(filename,'flow');
            end
            gyro_rate = obj.params.users.gyro_rate;
            flow_times = ((1:length(flow))-1)./obj.video.camera_model.frame_rate;
            gyro_times = ((1:length(obj.gyro.data))-1)./gyro_rate;
            % do estimation
            try
                filename = fullfile(obj.save_path,'time_offset.mat');
                if obj.dbg_level > 0
                    if exist(filename,'file')
                        load(filename,'time_offset');
                            
                        h = struct();
                        h.fig = figure('Name','Time Alignment', 'NumberTitle','off', 'Menubar','none', ...
                            'Pointer','cross', 'Resize','off', 'Position',[200 200 400 400]);
                        if ~mexopencv.isOctave()
                            %HACK: not implemented in Octave
                            movegui(h.fig, 'west');
                        end
                        gyro_org = vecnorm(obj.gyro.data);
                        h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
                        subplot(h.ax);
                        subplot(2,1,1);
                        plot(flow_times, flow/max(flow), 'r-','LineWidth',2);hold on;
                        plot(gyro_times, gyro_org/max(gyro_org), 'b-','LineWidth',2);grid on;
                        legend({'flow','gyro'});
                        title('Before Alignment');
                        subplot(2,1,2);
                        plot(flow_times+time_offset, flow/max(flow), 'r-','LineWidth',2);hold on;
                        plot(gyro_times, gyro_org/max(gyro_org), 'b-','LineWidth',2);grid on;
                        legend({'flow','gyro'});
                        title('After Alignment');
                    else
                        time_offset = sync_camera_gyro(flow, flow_times, obj.gyro.data, ...
                                            gyro_times, true, nlevels, obj.save_path);
                        if ~exist(obj.save_path,'dir')
                            mkdir(obj.save_path);
                        end
                        save(filename,'time_offset');
                    end
                end
            catch
                error('time alignment failure');
            end
            obj.params.initialized.time_offset = time_offset;
        end
        
    end
end