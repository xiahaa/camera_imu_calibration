classdef Slice < handle
%     """
%     Video slice module
%     """
    properties
        points;
        s;
        e;
        axis;
        angle;
        inliers;
    end
    methods
        function obj = Slice(s, e, points)
            obj.points = points;
            obj.s = s;
            obj.e = e;
            obj.axis = [];
            obj.angle = [];
            obj.inliers = [];
        end
        function axis = estimate_rotation(obj, camera, varargin)
            % """Estimate the rotation between first and last frame
            %
            % It uses RANSAC where the error metric is the reprojection error of the points
            % from the last frame to the first frame.
            %
            % Parameters
            % -----------------
            % camera : CameraModel
            % Camera model
            % ransac_threshold : float
            % Distance threshold (in pixels) for a reprojected point to count as an inlier
            % """
            if nargin == 3
                threshold = varargin{1};
            else
                thereshold = 7;
            end
            
            if isempty(obj.axis)
                x = obj.points{1}{:};
                y = obj.points{end}{:};
                inlier_ratio = 0.5;
                [R, t, dist, idx] = estimate_rotation_procrustes_ransac(x, y, ...
                    camera, ransac_threshold, inlier_ratio, false);
                if ~isempty(R)
                    [obj.axis, obj.angle] = rotation_matrix_to_axis_angle(R);
                    if obj.angle < 0 % Constrain to positive angles
                        obj.angle = -obj.angle
                        obj.axis = -obj.axis
                    obj.inliers = idx
                    end
                end
            end
            axis = obj.axis;
        end
    end
    methods(Static) 
        function slices = from_stream_randomly(video_stream, varargin)
            if nargin >= 2
                step_bounds = varargin{1};
            else
                step_bounds = [5,15];
            end
            if nargin >= 3
                length_bounds = varargin{2};
            else
                length_bounds = [2,15];
            end
            if nargin >= 4
                max_start = varargin{2};
            else
                max_start = inf;
            end
            if nargin >= 5
                min_distance = varargin{3};
            else
                min_distance = 10;
            end
            if nargin >= 6
                min_slice_points = varargin{4};
            else
                min_slice_points = 10;
            end
            if nargin >= 7
                do_plot = varargin{5};
            else
                do_plot = true;
            end
            l1 = step_bounds(2) - step_bounds(1) + 1;
            l2 = length_bounds(2) - length_bounds(1) + 1;
            %
            new_step = @() (randi(l1)-1+step_bounds(1));
            new_length = @() (randi(l2)-1+length_bounds(1));
            
            seq_frames = {};
            
            slices = {};
            
            % temp var for loop
            seq_start_points = [];
            next_seq_start = min(new_step(),max_start);
            next_seq_length = new_length();
            
            gftt_params = struct('max_corners', 400, 'quality_level', 0.07,'min_distance', min_distance);
            
            i = 1;
            while 1
                im = video_stream.read();
                if isempty(im), break; end
                im = cv2.cvtColor(im, 'RGB2GRAY');
                if next_seq_start <= i < next_seq_start + next_seq_length
                    seq_frames{end+1} = im;
                    if length(seq_frames) == next_seq_length
                        % detect features in the first frame
                        seq_start_points=feature_detection(seq_frames{1},gftt_params);
                        points, status = tracking.track_retrack(seq_frames,seq_start_points,0.8,false,true);
                        if length(status) >= min_slice_points
                            % consider this slice is good
                            s = Slice(next_seq_start,i,points);
                            slices{end+1}=s;
                            debuginfo = sprintf('%4d %3d %5d {%9d-%9d}', length(slices)-1, length(status), next_seq_start, i);
                            disp(debuginfo);
                        end
                        seq_frames=[];% clear for next slice
                        % random sample a new step and new length
                        next_seq_start = i+new_step();
                        next_seq_length = new_length();
                    end
                end
                i = i + 1;
            end
        end
        
        function samples = fill_sampling(slice_list, N)
            % """Given a list of slices,
            % draw N samples such that each slice
            % contributes as much as possible
            % Parameters
            % --------------------------
            % slice_list : list of Slice
            % List of slices
            % N : int
            % Number of samples to draw
            % """
            
            % a list of inlier numbers for all slices
            A = cellfun(@(s) (length(s.inliers)), slice_list);
            % the sum of all inliers of all slices
            N_max = sum(A);
            assert(N<=N_max);%the sample N could be over the maximum inlier number
            
            samples_from = zeros(length(A),1);%Number of samples to draw from each group
            remaining = N;
            
            while remaining > 0
                remaining_groups = find(samples_from-A);
                if remaining < length(remaining_groups)
                    remaining_groups = remaining_groups(randperm(length(remaining_groups)));
                    samples_from(remaining_groups(1:remaining_groups)) = samples_from(remaining_groups(1:remaining_groups)) + 1;
                else
                    % Give each group the allowed number of samples. Constrain to their max size.
                    to_each = max(1, int(remaining / length(remaining_groups)));
                    samples_from = samples_from + to_each;
                    id = samples_from>A;
                    samples_from(id) = A(id);
                end
                % Update remaining count
                remaining = int(N - sum(samples_from));
            end
            if remaining ~= 0
                error("Still {:d} samples left! This is an error in the selection.");
            end
            % Construct index list of selected samples
            samples = cell(1,length(slice_list));
            for i = 1:length(slice_list)
                if A(i) == samples_from(i)
                    samples{i} = slice_list{i}.inliers;
                elseif A(i) == 0
                    samples{i} = [];
                else
                    id = randperm(A(i));
                    samples{i} = slice_list{i}.inliers(id(1:n));
                end
            end
        end
    end
end

