classdef tracking
    methods(Static)
        function flow = frametoframe_track(imgseq, varargin)
            if nargin >= 2
                max_diff = varargin{1};
            else
                max_diff=60;
            end
            if nargin >= 3
                gftt_options = varargin{2};
            else
                gftt_options=struct();
            end
            if nargin >= 4
                do_plot = varargin{3};
            else
                do_plot=true;
            end
            flow=zeros(1,length(imgseq)-1);
            prev_img = imgseq{1};
            if size(prev_img,3) == 3
                prev_img = cv.cvtColor(prev_img, 'RGB2GRAY');
            end
            prev_img = cv.resize(prev_img,0.5,0.5);
            if do_plot
                sz = [640,480];
                h = struct();
                h.fig = figure('Name','frame to frame tracking', 'NumberTitle','off', 'Menubar','none', ...
                    'Pointer','cross', 'Resize','off', 'Position',[200 200 sz(2) sz(1)]);
                if ~mexopencv.isOctave()
                    %HACK: not implemented in Octave
                    movegui(h.fig, 'center');
                end
                h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
            end
            
            for i = 2:length(imgseq)
                img = imgseq{i};
                if size(img,3) == 3
                    img = cv.cvtColor(img, 'RGB2GRAY');
                end
                img = cv.resize(img, 0.5,0.5);
                
                % detect features in previous img
                prev_points = feature_detection(prev_img,gftt_options,true);

                % use optical flow tracking
                [curr_points,status,err] = cv.calcOpticalFlowPyrLK(prev_img, img, prev_points, lk_params{:});
                
                %
                valids = status ~= 0;
                distance = cellfun(@(a,b) norm(a-b), curr_points(valids), prev_points(valids));
                
                dm = mean(distance(distance<max_diff));
                
                if isnan(dm)
                   dm = 0; 
                end
                
                flow(i-1)=dm;
                prev_img = img;
                
                if do_plot == True
                    imgc = cv.cvtColor(img,'GRAY2RGB');
                    % resize
                    imgc = cv.resize(imgc,[640,480]);
                    imgc = cv.circle(imgc, corners1, 2, 'Thickness','Filled', 'Color',[255,0,0]);
                    imgc = cv.circle(imgc, corners2, 2, 'Thickness','Filled', 'Color',[0,255,255]);
                    tracks = cellfun(@(p1,p2) [p1; p2], prev_points(valids), curr_points(valids), 'UniformOutput',false);
                    imgc = cv.polylines(imgc, tracks, 'Closed',false, 'Color',[0,255,0]);
                    
                    try 
                        set(h.img, 'CData',imgc);
                    catch
                        h.img = imshow(imgc, 'Parent',h.ax);
                    end                    
                    drawnow
                end
            end
            close(h.fig);
        end
        
        function [tracks,track_status]=track(imgseq, initial_points, varargin)
            if nargin >= 3
                remove_bad = varargin{1};
            else
                remove_bad=false;
            end
            tracks = cell(1,length(imgseq));
            tracks(1) = initial_points;
            track_status = ones(1,size(initial_points{1},2));
            window_size = [5,5];
            
            prev_img = [];
            
            for i = 2:length(imgseq)
                img1 = imgseq{i-1};
                img2 = imgseq{i};
                %
                prev_ok_status = find(track_status == 1);
                % get previsou ok feature points
                prev_points = track{i-1}{prev_ok_status};
                % tracking
                [points, status, err]=cv.calcOpticalFlowPyrLK(img1,img2,prev_points,'WinSize',window_size);
                if all(status)
                    track_status(1:end) = 0
                    break;
                end
                %valid index
                valids = status ~= 0;
                now_ok_status = prev_ok_status(valids);
                tracks{i} = initial_points;
                tracks{i}{now_ok_status} = points{valids};
                track_status(prev_ok_status) = status;
            end
            
            if remove_bad
                final_ok = track_status ~= 0;
                tracks = tracks{:}{final_ok};
                track_status = track_status(final_ok);
            end
        end
        
        function track_retrack(imgseq,initial_points,varargin)
            if nargin >= 3
                max_retrack_distance = varargin{1};
            else
                max_retrack_distance=0.5;
            end
            if nargin >= 4
                keep_bad = varargin{2};
            else
                keep_bad=false;
            end
            if nargin >= 5
                do_plot = varargin{3};
            else
                do_plot=true;
            end
            
            [forward_track, forward_status] = track(imgseq,initial_points,false);
            [backward_track, backward_status] = track(imgseq{end:-1:1},forward_track{end},false);
            ok_status = forward_status * backward_status;
            % features in the first frame
            points1 = forward_track{1}{ok_status};
            % retracked features in the first frame (last in the backward mode)
            points2 = backward_track{end}{ok_status};
            retrack_distance = vecnorm(points2-points1);
            valids = retrack_distance < max_retrack_distance;
            final_status = ok_status(valids);
            
            if ~keep_bad
                final_status = final_status == 1;
                tracks = forward_track{:}{final_status};
                track_status = forward_status(final_status);                
            else
                tracks = forward_track;
                track_status = zeros(1, length(forward_status));
                track_status(final_status) = 1;
            end

            if do_plot
                sz = [640,480];
                fh = findobj( 'Type', 'Figure', 'Name', 'Slice Tracking' );
                if isempty(fh)
                    h = struct();
                    h.fig = figure('Name','', 'NumberTitle','off', 'Menubar','none', ...
                        'Pointer','cross', 'Resize','off', 'Position',[200 200 sz(2) sz(1)]);
                    if ~mexopencv.isOctave()
                        %HACK: not implemented in Octave
                        movegui(h.fig, 'center');
                    end
                    h.ax = axes('Parent',h.fig, 'Units','normalized', 'Position',[0 0 1 1]);
                else
                    h = fh;
                end
                
                imgc = cv.cvtColor(imgseq{end},'GRAY2RGB');
                ok = track_status == 1;
                corners1 = tracks{1}{ok};
                corners2 = tracks{end}{ok};
                
                % resize
                imgc = cv.resize(imgc,[640,480]);
                imgc = cv.circle(imgc, corners1, 2, 'Thickness','Filled', 'Color',[255,0,0]);
                imgc = cv.circle(imgc, corners2, 2, 'Thickness','Filled', 'Color',[0,255,255]);
                imgc = cv.polylines(imgc, tracks{:}{ok}, 'Closed',false, 'Color',[0,255,0]);

                try 
                    set(h.img, 'CData',imgc);
                catch
                    h.img = imshow(imgc, 'Parent',h.ax);
                end                    
                drawnow;
                pause(0.1);
            end
        end
    end
end
