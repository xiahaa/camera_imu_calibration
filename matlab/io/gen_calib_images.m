function gen_calib_images(videofile)
    cap = VideoReader(videofile);
    vars = zeros(cap.NumberofFrames,1);
    [dir,name,ext]=fileparts(videofile);
    if exist(fullfile(dir,strcat(name,'_vars.mat')),'file') == 0
        for i = 1:cap.NumberofFrames
            frame = read(cap,i);
            frame_small = cv.resize(frame,[640,480]);
            frame_gray = cv.cvtColor(frame_small,'RGB2GRAY');
            frame_lap = cv.Laplacian(frame_gray);
            vars(i) = std(double(frame_lap(:)));
        end
    else
        load(fullfile(dir,strcat(name,'_vars.mat')));
    end

    meanvar = mean(vars);
    stdvars = std(vars);
    
    index = find((vars - meanvar) > 0.5 * stdvars);
    
    if exist(fullfile(dir,name),'dir') == 0
        mkdir(fullfile(dir,name));
    end
    
    patternsize = [7,5];
    
    old_frame_gray = [];
    old_corners = [];
    for i = 1:length(index)
        frame = read(cap,index(i));
        new_frame_small = cv.resize(frame,[640,480]);
        new_frame_gray = cv.cvtColor(new_frame_small,'RGB2GRAY');
        [new_corners,patternfound] = cv.findChessboardCorners(new_frame_gray, patternsize, ...
         'AdaptiveThresh',true, 'NormalizeImage',true, 'FastCheck',true);
        if patternfound 
            if isempty(old_frame_gray)
                filename = sprintf('%s/%s/%06d.bmp',dir,name,index(i));
                cv.imwrite(filename,frame);
                old_frame_gray = new_frame_gray;
                old_corners = new_corners;
            else
                area1 = (new_corners{end}(1) - new_corners{1}(1)) * (new_corners{end}(2) - new_corners{1}(2));
                area2 = (old_corners{end}(1) - old_corners{1}(1)) * (old_corners{end}(2) - old_corners{1}(2));
                if abs(area1/area2-1) < 0.5
                    displacement = cellfun(@(t1,t2) (sum(abs(t1-t2))), new_corners, old_corners);
                    displacement = mean(displacement);
                    if displacement < 40
                        continue;
                    end
                else
                    if area1 / (size(new_frame_gray,1)*size(new_frame_gray,2)) < 0.3
                        continue;
                    end
                    displacement = cellfun(@(t1,t2) (sum(abs(t1-t2))), new_corners, old_corners);
                    displacement = mean(displacement);
                    if displacement < 50
                        continue;
                    end
                end
                imgc = cat(2,old_frame_gray,new_frame_gray);
                imshow(imgc);hold on;
                cmap = jet(patternsize(1)*patternsize(2));
                for ii = 1:length(new_corners)
                    x = [old_corners{ii}(1), new_corners{ii}(1)+size(old_frame_gray,2)];
                    y = [old_corners{ii}(2), new_corners{ii}(2)];
                    plot(x,y,'-','Color',cmap(ii,:));
                end
                hold off;
                pause(1);
                
                filename = sprintf('%s/%s/%06d.bmp',dir,name,index(i));
                cv.imwrite(filename,frame);
                old_frame_gray = new_frame_gray;
                old_corners = new_corners;
            end
        else
            continue;
        end
    end
    close all;
end