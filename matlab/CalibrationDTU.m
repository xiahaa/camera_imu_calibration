classdef CalibrationDTU < handle
    properties
        video;
        imugps;
        slice;
        params;
        dbg_level;
        save_path;
        result;
        R2;
        master_state;
        t2;
    end
    methods
        function obj = CalibrationDTU(video,imugps,debug_level,save_path)
            obj.video = video;
            obj.imugps = imugps;
            obj.slice = [];
            obj.params = struct('users',struct(),'initialized',struct(),'calibration',struct());
            obj.dbg_level = 5;
            obj.save_path = save_path;
            obj.result = zeros(8,1);
            obj.master_state = [];
            obj.t2 = [];
        end
        function initialize(obj,gyro_rate,varargin)
            if nargin > 2
                obj.master_state = varargin{1};
            end
            
            if nargin > 3
                obj.t2 = varargin{2};
            end          
            
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
            
%             try
            R2 = find_R2(obj);
            t2 = find_t2(obj);
%             R1 = find_R1(obj);
    
            %% todo, right now use the cheesboard solution is more accurate
            if ismac
                 idcs   = strfind(obj.save_path,'/');
            else
                idcs   = strfind(obj.save_path,'\');
            end
            newdir = obj.save_path(1:idcs(end-1)-1);
            R1 = load(fullfile(newdir,'R1.mat'));
            filename = fullfile(obj.save_path,'extrinsics.mat'); 
            save(filename,'R2','R1','t2');
            
%             catch
                
%             end
            
            %% now estimate relative rotation 
%             R = obj.find_initial_rotation(); % TODO
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
        
        function [n0, tau] = video_time_to_gopro_gyro(obj,t)
            n0 = find((obj.video.gopro_gyro(1,:)-t)>0,1)-1;
            tau = (t - obj.video.gopro_gyro(1,n0)) * obj.video.gopro_f;
        end
        
        % find t2 is not so reliable because the commercial software they
        % use is closed-source, so there is no means to exactly transform
        % from latitude, longitude, altitude to local tangent frame. Large
        % error may be generated by different transformation algorithms and
        % using different earth model. 
        function t2 = find_t2(obj)
            %% lsq to find t2, use RANSAC
            wgs84 = wgs84Ellipsoid;
            if isempty(obj.master_state)
                lat0 = 55+46/60+59.62892/3600;
                long0 = 12+30/60+57.55967/3600;
                h0 = 97.717;
            else
                lat0 = obj.master_state.lat(1)+obj.master_state.lat(2)/60+obj.master_state.lat(3)/3600;
                long0 = obj.master_state.lon(1)+obj.master_state.lon(2)/60+obj.master_state.lon(3)/3600;
                h0 = obj.master_state.h;
            end
            [lly,llx,llz]=geodetic2ned(obj.video.gopro_gps(2,:),obj.video.gopro_gps(3,:),obj.video.gopro_gps(4,:),lat0,long0,h0,wgs84);
            llz = -llz;
            % find start point
            [minval,minid] = min(abs(obj.imugps.time-(obj.video.gopro_gps(1,1)+obj.params.initialized.time_offset)));
            if minval < 0.01
                % relative to start point
                llx = llx - llx(1);
                lly = lly - lly(1);
                llz = llz - llz(1);
                % make imugps relative to start point
                imugps_p = obj.imugps.p-obj.imugps.p(:,minid);
            end
            
            vel_mag = (obj.video.gopro_gps(5,:));
            id = vel_mag > (mean(vel_mag)+0.3*std(vel_mag));
            gps_temp = obj.video.gopro_gps(:,id);
            
            llx = llx(id);lly = lly(id);llz = llz(id);
            
            idalign = 1;
            x = zeros(4,length(gps_temp));
            y = zeros(4,length(gps_temp));
            rpy = zeros(3,length(gps_temp));
            vgn = vecnorm(obj.imugps.vg);
            for i = 1:length(gps_temp)
                t1 = gps_temp(1,i);
                [minval,minid] = min(abs(obj.imugps.time-(t1+obj.params.initialized.time_offset)));
                if minval < 0.01
                    % valid
                    x(:,i) = [llx(i);lly(i);llz(i);vel_mag(i)];
                    y(:,i) = [imugps_p(:,minid);vgn(minid)];
                    rpy(:,i) = [obj.imugps.yaw(minid);obj.imugps.pitch(minid);obj.imugps.roll(minid)];
                    idalign = minid;
                else
                    if idalign == length(obj.imugps.time)
                        break;
                    end
                end
            end
            x(:,i+1:end)=[];
            y(:,i+1:end)=[];
            rpy(:,i+1:end)=[];
            
            id = (vecnorm(x(1:3,:)-y(1:3,:))<0.5);
            x = x(:,id); y = y(:,id);
            
            h = struct();
            h.fig = figure('Name','t2 Alignment Data Preparation', 'NumberTitle','off', 'Menubar','none', ...
                        'Pointer','cross', 'Resize','on', 'Position',[100 100 400 400]);
            if ~mexopencv.isOctave()
                %HACK: not implemented in Octave
                movegui(h.fig, 'east');
            end
            h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
            subplot(h.ax);
            subplot(5,1,1);
            plot(obj.imugps.time,vecnorm(obj.imugps.vg));hold on;grid on;
            plot(obj.video.gopro_gps(1,:)+obj.params.initialized.time_offset,obj.video.gopro_gps(5,:));
            plot(gps_temp(1,:)+obj.params.initialized.time_offset,gps_temp(5,:),'g+')

            subplot(5,1,2);
            plot(x(1,:));hold on;
            plot(y(1,:));grid on;
            
            subplot(5,1,3);
            plot(x(2,:));hold on;
            plot(y(2,:));grid on;
            
            subplot(5,1,4);
            plot(x(3,:));hold on;
            plot(y(3,:));grid on;
            
            subplot(5,1,5);
            plot(x(4,:));hold on;
            plot(y(4,:));grid on;
            
            %% now we have data
            b = x(1:3,:) - y(1:3,:); b = vec(b);
            
            R = angle2dcm(rpy(1,id),rpy(2,id),rpy(3,id));
            A = reshape(permute(R,[1,3,2]),[],3);
            
            if isempty(obj.t2)
                manual_measure = [0;0.197;-0.080];
            else
                manual_measure = obj.t2;
            end
            bnd = [0.010;0.010;0.010];
            lb = manual_measure - bnd;
            ub = manual_measure + bnd;
            t2 = quadprog(2*(A'*A),-2*b'*A,[],[],[],[],lb,ub);
        end
        
        function R2 = find_R2(obj)
            %% done
            gyro_mag = vecnorm(obj.video.gopro_gyro(2:4,:));
            id = gyro_mag > (mean(gyro_mag)+0.5*std(gyro_mag));
            gyro_1 = obj.video.gopro_gyro(:,id);
            x = zeros(3,length(gyro_1));
            y = zeros(3,length(gyro_1));
            idalign = 1;
            invalid = [];
            for i = 1:length(gyro_1)
                t1 = gyro_1(1,i);
                [minval,minid] = min(abs(obj.imugps.time-(t1+obj.params.initialized.time_offset)));
                if minval < 0.01
                    % valid
                    x(:,i) = gyro_1(2:4,i);
                    y(:,i) = obj.imugps.w(:,minid);
                    idalign = minid;
                else
                    if idalign == length(obj.imugps.time)
                        break;
                    else
                        invalid(end+1) = i;
                    end
                end
            end
%             x(:,i+1:end)=[];
%             y(:,i+1:end)=[];
            x(:,invalid) = [];
            y(:,invalid) = [];
            
            if length(x) < 2
                error('Hand-eye calibration requires >= 2 rotations');
            end
        
            % [0] since we only cares about rotation
            model_func = @(d) (procrustes(d(1:3,:), d(4:6,:), false));
        
            function err=eval_func(model, d)
                X=d(1:3,:);
                Y=d(4:6,:);
                Rest = model;
                Xhat = Rest*Y;
                err = X-Xhat;
                err = vecnorm(err);
            end 
            
            model_points = 2; % select to not use minimal case
            threshold = 0.2;% tunable
            
            %% only for debug            
            data = [x;y];
            R = adaRANSAC(model_func, @eval_func, data, model_points, threshold, true);
            [n, theta] = rotation_matrix_to_axis_angle(R);
            so3 = theta*n;
            obj.params.initialized.rot2_x = so3(1);
            obj.params.initialized.rot2_y = so3(2);
            obj.params.initialized.rot2_z = so3(3);
            disp([so3]);
            R2 = R;
            
            align_gyro = R'*obj.video.gopro_gyro(2:4,:);
            
            h = struct();
            h.fig = figure('Name','R2 Alignment', 'NumberTitle','off', 'Menubar','none', ...
                        'Pointer','cross', 'Resize','on', 'Position',[100 100 400 400]);
            if ~mexopencv.isOctave()
                %HACK: not implemented in Octave
                movegui(h.fig, 'east');
            end
            h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
            subplot(h.ax);
            subplot(3,1,1);
            plot(obj.video.gopro_gyro(1,:)+obj.params.initialized.time_offset, align_gyro(1,:), 'r-','LineWidth',2);hold on;
            plot(obj.imugps.time, obj.imugps.w(1,:), 'b-.','LineWidth',1.5);grid on;
            legend({'gopro gyro','gyro'});
            subplot(3,1,2);
            plot(obj.video.gopro_gyro(1,:)+obj.params.initialized.time_offset, align_gyro(2,:), 'r-','LineWidth',2);hold on;
            plot(obj.imugps.time, obj.imugps.w(2,:), 'b-.','LineWidth',1.5);grid on;
            legend({'gopro gyro','gyro'});
            subplot(3,1,3);
            plot(obj.video.gopro_gyro(1,:)+obj.params.initialized.time_offset, align_gyro(3,:), 'r-','LineWidth',2);hold on;
            plot(obj.imugps.time, obj.imugps.w(3,:), 'b-.','LineWidth',1.5);grid on;
            legend({'gopro gyro','gyro'});
            
            %% temp code
%             R = eye(3);
%             t = obj.video.gopro_gyro(1,1);
%             Rs = zeros(3,3,size(obj.video.gopro_gyro,2));
%             for i = 2:size(obj.video.gopro_gyro,2)
%                 dt = obj.video.gopro_gyro(1,i) - t;
%                 t = obj.video.gopro_gyro(1,i);
%                 Rs(:,:,i) = R * expm(skewm(obj.video.gopro_gyro(2:4,i))*dt);
%                 R = Rs(:,:,i);
%             end
%             [y,p,r] = dcm2angle(Rs);
%             figure
%             subplot(3,1,1);
%             plot(obj.video.gopro_gyro(1,:)+obj.params.initialized.time_offset, y, 'r-','LineWidth',2);hold on;
%             plot(obj.imugps.time, obj.imugps.yaw, 'b-.','LineWidth',1.5);grid on;
%             legend({'gopro gyro','gyro'});
%             subplot(3,1,2);
%             plot(obj.video.gopro_gyro(1,:)+obj.params.initialized.time_offset, p, 'r-','LineWidth',2);hold on;
%             plot(obj.imugps.time, obj.imugps.pitch, 'b-.','LineWidth',1.5);grid on;
%             legend({'gopro gyro','gyro'});
%             subplot(3,1,3);
%             plot(obj.video.gopro_gyro(1,:)+obj.params.initialized.time_offset, r, 'r-','LineWidth',2);hold on;
%             plot(obj.imugps.time, obj.imugps.roll, 'b-.','LineWidth',1.5);grid on;
%             legend({'gopro gyro','gyro'});
        end
        
        function R = find_R1(obj)
%             q_list = integrate_gyro_quaternion(gyro_ts, gyro_data);
            dt = mean(diff(obj.video.gopro_gyro(1,:)));
            
            % double check integration is correct
%             gopro_gyro_in_imu = obj.R2'*obj.video.gopro_gyro(2:4,:);
%             q1 = integrate_gyro_quaternion_uniform(gopro_gyro_in_imu, dt, []);% integration to get the imu rotation
%             q2 = [obj.imugps.q]';
%             R=quat2dcm(q2);
%             [~,minid] = min(abs(obj.imugps.time-(obj.video.gopro_gyro(1,1)+obj.params.initialized.time_offset)));
%             R0 = R(:,:,minid);
%             for i = 1:length(q2)
%                 R(:,:,i) = (R0'*R(:,:,i))';
%             end
%             q = dcm2quat(R);
            
            q = integrate_gyro_quaternion_uniform(obj.video.gopro_gyro(2:4,:), dt, []);% integration to get the rotation, this rotation is the rotation from initial frame to current frame
            
            len = length(obj.slice);
            video_axes = zeros(3,len);
            gyro_axes = zeros(3,len);
            final_len = 0;
            
            ransac_thereshold = 2;
            for k = 1:len
                % # estimate rotation here
                % # estimate the rotation for slice
                % # to make sure within this slice, there is
                % # a significant rotation
                s = obj.slice{k};
                % this R is from end frame to initial frame
                s.estimate_rotation(obj.video.camera_model,ransac_thereshold);
                if isempty(s.axis)
                    continue;
                end
%                 if ~(s.angle > 0)
%                     continue;
%                 end
                
                tmp = 1 / obj.video.camera_model.frame_rate;
                
                t1 = (s.s-1) * tmp + obj.video.frame_ts;
                [n1,~] = obj.video_time_to_gopro_gyro(t1);
                t2 = (s.e-1) * tmp + obj.video.frame_ts;
                [n2,~] = obj.video_time_to_gopro_gyro(t2);
                
                try
                    qx = q(:,n1);
                    qy = q(:,n2);
                catch
                    continue;
                end
                
                % to rotation matrix
                Rx = quat_to_rotation_matrix(qx)';% transpose -> rotation matrix from current to initial
                Ry = quat_to_rotation_matrix(qy)';% transpose -> rotation matrix from current to initial
                dR = Rx'*Ry;% 
                [v, theta] = rotation_matrix_to_axis_angle(dR);
                if theta < 0
                    v = -v;
                end
                
                if abs(s.angle - theta) > 5*pi/180;
                    continue;
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
        
        function R = find_initial_rotation(obj)
            q = obj.imugps.q;% imu quaternion
                        
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
                % this R is from end frame to initial frame
                s.estimate_rotation(obj.video.camera_model,ransac_thereshold);
                if isempty(s.axis)
                    continue;
                end
%                 if ~(s.angle > 0)
%                     continue;
%                 end
                
                tmp = 1 / obj.video.camera_model.frame_rate;
                
                t1 = (s.s-1) * tmp + obj.video.frame_ts;
                [n1,~] = obj.video_time_to_gyro_sample(t1);
                t2 = (s.e-1) * tmp + obj.video.frame_ts;
                [n2,~] = obj.video_time_to_gyro_sample(t2);
                
                try
                    qx = q(:,n1);
                    qy = q(:,n2);
                catch
                    continue;
                end
                
                % to rotation matrix
                Rx = quat_to_rotation_matrix(qx)';% transpose -> rotation matrix from body to ground
                Ry = quat_to_rotation_matrix(qy)';% transpose -> rotation matrix from body to ground
                dR = Rx'*Ry;% 
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
                    load(filename,'flow','flow_time');
                    id = flow_time >= obj.video.frame_ts;
                    flow = flow(id);
                else
                    flow = obj.video.estimate_flow(true);
                    flow = reshape(flow,1,[]);
                    flow_time = (1:length(flow))/obj.video.camera_model.frame_rate + obj.video.frame_ts;
                    if ~exist(obj.save_path,'dir')
                        mkdir(obj.save_path);
                    end
                    save(filename,'flow','flow_time');
                end
            else
                flow = obj.video.estimate_flow(true);
                flow = reshape(flow,1,[]);
                flow_time = (1:length(flow))/obj.video.camera_model.frame_rate + obj.video.frame_ts;
                if ~exist(obj.save_path,'dir')
                    mkdir(obj.save_path);
                end
                save(filename,'flow','flow_time');
            end
            
            %% 
            % sync gopro with imugps as:                        
            %           t0 = obj.frame_ts, talign = t-t0 + time_offset
            %
            %%
            if isempty(obj.video.gopro_gyro)
                %% no parsed gyro data
                gyro_rate = obj.params.users.gyro_rate;
                flow_times = ((1:length(flow))-1)./obj.video.camera_model.frame_rate;
                gyro_times = ((1:obj.imugps.num_samples)-1)./gyro_rate;
                
                % do estimation
                try
                    filename = fullfile(obj.save_path,'time_offset.mat');
                    if obj.dbg_level > 0
                        if exist(filename,'file')
                            load(filename,'time_offset');
                        else
                            time_offset = sync_camera_gyro(flow, flow_times, obj.imugps.w, ...
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
            else
                gyro_rate = obj.params.users.gyro_rate;
                gyro_mag_gopro = sqrt(obj.video.gopro_gyro(2,:).^2+obj.video.gopro_gyro(3,:).^2+obj.video.gopro_gyro(4,:).^2);

                filename = fullfile(obj.save_path,'time_offset.mat');
                if obj.dbg_level > 0
                    if exist(filename,'file')
                        load(filename,'time_offset','time_offset_to_imu_local');
                    else
                        %% if gopro data can be parsed, then use gopro integrated gyro to sync with external imu gps
                        gyro_times_gopro = obj.video.gopro_gyro(1,:) - obj.video.gopro_gyro(1,1);
                        gyro_times = ((1:obj.imugps.num_samples)-1)./gyro_rate;

                        time_offset = sync_camera_gyro(gyro_mag_gopro, gyro_times_gopro, obj.imugps.w, gyro_times, false, nlevels, obj.save_path);
                        
                        % correct to imugps local time
                        time_offset_to_imu_local = obj.imugps.time(1) - obj.video.gopro_gyro(1,1) + time_offset;
                        
                        if ~exist(obj.save_path,'dir')
                            mkdir(obj.save_path);
                        end
                        save(filename,'time_offset','time_offset_to_imu_local');
                    end
                end
                
                %% draw aligned 1-aligned gyro, 2-gyro and flow, 3-gps horizontal position.
                if true
                    gyro_org = vecnorm(obj.imugps.w);
                    h = struct();
                    h.fig = figure('Name','Time Alignment', 'NumberTitle','off', 'Menubar','none', ...
                        'Pointer','cross', 'Resize','on', 'Position',[200 200 400 400]);
                    if ~mexopencv.isOctave()
                        %HACK: not implemented in Octave
                        movegui(h.fig, 'east');
                    end
                    h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
                    subplot(h.ax);
                    subplot(4,1,1);
                    plot(obj.video.gopro_gyro(1,:), gyro_mag_gopro/max(gyro_mag_gopro), 'r-','LineWidth',2);hold on;
                    plot(obj.imugps.time, gyro_org/max(gyro_org), 'b-','LineWidth',2);grid on;
                    legend({'gopro gyro','gyro'});
                    title('Before Alignment');
                    subplot(4,1,2);
                    plot(obj.video.gopro_gyro(1,:)+time_offset_to_imu_local, gyro_mag_gopro/max(gyro_mag_gopro), 'r-','LineWidth',2);hold on;
                    plot(obj.imugps.time, gyro_org/max(gyro_org), 'b-','LineWidth',2);grid on;
                    legend({'gopro gyro','gyro'});
                    title('After Alignment');
                    
                    %% more results
%                     flow_times = ((1:length(flow)))./obj.video.camera_model.frame_rate;
%                     flow_times = flow_times + obj.video.frame_ts;
                    flow_times=flow_time;

                    subplot(4,1,3);
                    plot(flow_times+time_offset_to_imu_local, flow/max(flow), 'r-','LineWidth',2);hold on;
                    plot(obj.imugps.time, gyro_org/max(gyro_org), 'b-','LineWidth',2);grid on;
                    legend({'flow','gyro'});
                    title('After Alignment');
                    
                    subplot(4,1,4);
                    gps_times = obj.video.gopro_gps(1,:);
                    vgn = vecnorm(obj.imugps.vg);
                    
                    plot(gps_times+time_offset_to_imu_local, obj.video.gopro_gps(6,:), 'r-','LineWidth',2);hold on;
                    plot(obj.imugps.time, vgn, 'b-','LineWidth',2);grid on;
                    legend({'gopro gyro','gyro'});
                    title('After Alignment');
                end
            end
            
            obj.params.initialized.time_offset = time_offset_to_imu_local;
        end
        
    end
end