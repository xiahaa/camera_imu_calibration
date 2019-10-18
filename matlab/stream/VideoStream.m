classdef VideoStream < handle
    properties
        flow;
        flow_mode;
        camera_model;
        cap;
        filename;
        start_time;
        duration;
        failcnt;
        frame_rotation;
        frame_ts;
        %% special data if you can parse gopro data
        gopro_gps;
        gopro_gyro;
        gopro_acc;
        num;
        gopro_f;
    end
    methods
        function obj = VideoStream(camera_model, flow_mode)
            obj.flow=[];
            obj.flow_mode=flow_mode;
            obj.camera_model=camera_model;
            obj.cap = [];
            obj.filename=[];
            obj.start_time=[];
            obj.duration=[];
            obj.frame_rotation=[];
            obj.frame_ts = [];
            %
            obj.gopro_gps = [];
            obj.gopro_gyro = [];
            obj.gopro_acc = [];
            obj.gopro_f = [];
        end
        
        function imgpoints = project(obj, points)
            %             """Project 3D points to image coordinates.
            %
            %             This projects 3D points expressed in the camera coordinate system to image points.
            %
            %             Parameters
            %             --------------------
            %             points : (3, N) ndarray
            %                 3D points
            %
            %             Returns
            %             --------------------
            %             image_points : (2, N) ndarray
            %                 The world points projected to the image plane of the camera used by the stream
            %             """
        	imgpoints = obj.camera_model.project(points);
        end
        
        function num = get_frame_count(obj)
            try
                num = obj.cap.get('FrameCount');
            catch
                num = obj.num;
            end
        end
        
        function points=unproject(obj, image_points)
            %         """Find (up to scale) 3D coordinate of an image point
            %
            %         This is the inverse of the `project` function.
            %         The resulting 3D points are only valid up to an unknown scale.
            %
            %         Parameters
            %         ----------------------
            %         image_points : (2, N) ndarray
            %             Image points
            %
            %         Returns
            %         ----------------------
            %         points : (3, N) ndarray
            %             3D coordinates (valid up to scale)
            %         """
            points = obj.camera_model.unproject(image_points);
        end
        
        function from_file(obj,filename,varargin)
            if nargin >= 3
                obj.start_time=varargin{1};
            else
                obj.start_time = 0;
            end
            if nargin >= 4
                obj.duration=varargin{2};
            else
                obj.duration = inf;
            end
            obj.filename = filename;
            try
                obj.cap = cv.VideoCapture(obj.filename);
                assert(obj.cap.isOpened())
                % OpenCV does something really stupid: to set the frame we need to set it twice and query in between
                t = obj.start_time * 1000; % turn to milliseconds
                obj.cap.set('PosMsec', t);% equivalent to use CV_CAP_PROP_POS_MSEC
                obj.cap.read();
                obj.cap.set('PosMsec', t);
                obj.failcnt = 0;
                % get frame timestamps
                [~, t] = obj.read();
                obj.frame_ts = t/1e3;% local clocker mesc to s, this will be the start time of the flow
            catch
                obj.cap = VideoReader(obj.filename);
                obj.failcnt = 0;
                obj.num = obj.cap.NumberofFrames;
                obj.cap = VideoReader(obj.filename,'CurrentTime',obj.start_time);
                [~, t] = obj.read();
                obj.frame_ts = t;% local clocker mesc to s, this will be the start time of the flow
            end
%             while true
%                 [~,t] = obj.read();
%                 if isempty(t)
%                     break;
%                 end
%                 obj.frame_ts = [obj.frame_ts t];
%             end

            %% special part if you can parse gopro data
            if nargin >= 5
                imu=load(varargin{3});
                obj.gopro_gyro = imu.gopro_imu(:,5:8)';
                obj.gopro_gyro(1,:)=obj.gopro_gyro(1,:)*1e-3;%mesc to s
                obj.gopro_acc = imu.gopro_imu(:,1:4)';
                obj.gopro_acc(1,:)=obj.gopro_acc(1,:)*1e-3;%mesc to s
                id = obj.gopro_gyro(1,:) > obj.start_time;
                obj.gopro_gyro = obj.gopro_gyro(:,id);
                obj.gopro_acc = obj.gopro_acc(:,id);
                
                obj.gopro_f = 1./mean(diff(obj.gopro_gyro(1,:)));
            end
            if nargin >= 6
                gps=load(varargin{4});
                obj.gopro_gps = gps.gopro_gps';
                obj.gopro_gps(1,:)=obj.gopro_gps(1,:)*1e-3;
                id = obj.gopro_gps(1,:) > obj.start_time;
                obj.gopro_gps = obj.gopro_gps(:,id);
            end
        end
        
        function varargout = read(obj)
            if ismac
                t2 = obj.start_time * 1000 + obj.duration*1000.0; %if self.duration is not None else None
                t = obj.cap.get('PosMsec');
                frame=[];
                if t < t2
                    try
                        frame = obj.cap.read();
                    catch
                        warning('Failed to read frame');
                        obj.failcnt = obj.failcnt + 1;
                        if obj.failcnt > 20
                            error('Failed to read frame up to 20 times~~~');
                        end
                    end
                end
            else
                t2 = obj.start_time + obj.duration; %if self.duration is not None else None
                t = obj.cap.CurrentTime;
                if hasFrame(obj.cap) && t < t2
                    frame = obj.cap.readFrame();
                    t = obj.cap.CurrentTime;
                else
                    frame = [];
                    t = [];
                end
            end
            varargout{1} = frame;
            if isempty(frame)
                t = [];
            end
            varargout{2} = t;
        end
                    
        function play(obj)
            sz = [320,240];
            h = struct();
            h.fig = figure('Name','Raw Video', 'NumberTitle','off', 'Menubar','none', ...
                'Pointer','cross', 'Resize','off', 'Position',[200 200 sz(2) sz(1)]);
            if ~mexopencv.isOctave()
                %HACK: not implemented in Octave
                movegui(h.fig, 'center');
            end
            h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
            
            %% temporary code
            fid = fopen('framestamps.txt','w');
            t = 0;
            
            [frame,t] = obj.read();
            img = cv.resize(frame,[320,240]);
            h.img = imshow(img, 'Parent',h.ax);

            while true
%                 t = t + 1/30.0;
                fprintf(fid,'%6.6f\n',t/1e3);
                [frame,t] = obj.read();
                if isempty(frame)
                    break;                    
                end
                img = cv.resize(frame,[320,240]);
                set(h.img, 'CData',img)
                drawnow
            end
            fclose(fid);
            
            close(h.fig);
            
            % reset 
            if ismac
                t = obj.start_time * 1000; % turn to milliseconds
                obj.cap.set('PosMsec', t);% equivalent to use CV_CAP_PROP_POS_MSEC
                obj.cap.read();
                obj.cap.set('PosMsec', t);
                obj.failcnt = 0;
            else
                obj.cap = VideoReader(obj.filename,'CurrentTime',obj.start_time);
                obj.failcnt = 0;
            end
        end
        
        function flow = estimate_flow(obj, do_plot)
            if isempty(obj.flow)
                disp("Generating flow. This can take minutes depending on video length");
                if strcmp(obj.flow_mode,'rotation')
                    obj.generate_frame_to_frame_rotation(do_plot);
                elseif strcmp(obj.flow_mode,'optical')
                    obj.flow = tracking.video_track(obj);
                else % todo, use essential
                    error('Not yet implemented!');
                end
                flow = obj.flow;
            end
        end
        
        function est_frame_to_frame_rotation(obj, do_plot)
            rotation = zeros(3,1e6);
            weights = zeros(1,1e6);
            k = 0;
            
            GFTT_PARAMS.max_corners = 500;
            GFTT_PARAMS.quality_level = 0.07;
            GFTT_PARAMS.min_distance = 10;
            
            img1 = obj.read();
            assert(~isempty(img1));
            prev = cv.cvtColor(img1, 'RGB2GRAY');
            while true
                img2 = obj.read();
                if isempty(img2), break; end
                curr = cv.cvtColor(img2, 'RGB2GRAY');
                % detect
                initial_pts = feature_detection(prev, GFTT_PARAMS);
                % track-retrack
                [pts, ~] = tracking.track_retrack({prev, curr}, initial_pts, do_plot);
                % 
                X = pts{1};
                Y = pts{end};
                threshold = 2.0;
                % estimate rotation
                [R, ~, err, inliers] = estimate_rotation_procrustes_ransac(X, Y, self.camera_model, threshold);
                % 
                if isempty(R)
                    weight = 0;
                    angle = 0;
                    r = np.zeros(3);
                else
                    weight = (1.0 * length(inliers)) / length(pts{1});
                    [axis, angle] = rotation_matrix_to_axis_angle(R);
                    r = axis * angle;
                end
                rotation(:,k+1)=r;
                weights(1,k+1)=weight;
                k = k + 1;
            end
            rotation(:,k+1:end) = [];
            weights(:,k+1:end) = [];
            
            % Scale from rad/frame to rad/s
            rotation = rotation.*obj.camera_model.frame_rate;
            % Remove and interpolate bad values
            threshold = 0.2;
            mask = weights > threshold;
            x = 1:length(weights);
            
            rotation(1, ~mask) = interp1(x(mask), rotation(1, mask), x(~mask));
            rotation(2, ~mask) = interp1(x(mask), rotation(2, mask), x(~mask));
            rotation(3, ~mask) = interp1(x(mask), rotation(3, mask), x(~mask));
              
            obj.frame_rotation = rotation;
            obj.flow = vecnorm(rotation);
        end
        
        function est_frame_to_frame_rotation_from_essential(obj, do_plot)
            rotation = zeros(3,1e6);
            weights = zeros(1,1e6);
            k = 0;
            
            GFTT_PARAMS.max_corners = 300;
            GFTT_PARAMS.quality_level = 0.07;
            GFTT_PARAMS.min_distance = 10;
            
            img1 = obj.read();
            assert(~isempty(img1));
            prev = cv.cvtColor(img1, 'RGB2GRAY');
            while true
                img2 = obj.read();
                if isempty(img2), break; end
                curr = cv.cvtColor(img2, 'RGB2GRAY');
                % detect
                initial_pts = feature_detection(prev, GFTT_PARAMS);
                % track-retrack
                [pts, ~] = tracking.track_retrack({prev, curr}, initial_pts, do_plot);
                % 
                p1 = pts{1};
                p2 = pts{end};
                % undistort
                p1 = self.camera_model.undistort(p1);
                p2 = self.camera_model.undistort(p2);
                % findessential
                [E, mask] = cv.findEssentialMat(p1, p2, 'Method','Ransac');
                % decompose
                [R, t, ~, ~] = cv.recoverPose(E, p1, p2, 'CameraMatrix',eye(3), 'Mask',mask);
                % 
                if isempty(R)
                    weight = 0;
                    angle = 0;
                    r = np.zeros(3);
                else
                    weight = (1.0 * sum(mask==1)) / length(pts{1});
                    [axis, angle] = rotation_matrix_to_axis_angle(R);
                    r = axis * angle;
                end
                rotation(:,k+1)=r;
                weights(1,k+1)=weight;
                k = k + 1;
            end
            rotation(:,k+1:end) = [];
            weights(:,k+1:end) = [];
            
            % Scale from rad/frame to rad/s
            rotation = rotation.*obj.camera_model.frame_rate;
            % Remove and interpolate bad values
            threshold = 0.2;
            mask = weights > threshold;
            x = 1:length(weights);
            
            rotation(1, ~mask) = interp1(x(mask), rotation(1, mask), x(~mask));
            rotation(2, ~mask) = interp1(x(mask), rotation(2, mask), x(~mask));
            rotation(3, ~mask) = interp1(x(mask), rotation(3, mask), x(~mask));
              
            obj.frame_rotation = rotation;
            obj.flow = vecnorm(rotation);
        end
    end
end





    