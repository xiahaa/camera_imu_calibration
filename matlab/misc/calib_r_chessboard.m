clc;close all; clear all;
% this file trys to calibrate the relative rotation between gopro image and
% gopro imu based on a chessboard.

fid = fopen('calib_config.json', 'r');
str = fread(fid, '*char').';
fclose(fid);
J = jsondecode(str);

foldername = J.folder.foldername;%'D:\dtu\data\calibration\gopro\1219\GH014320';%'D:\dtu\data\calibration\gopro\GH014310';

files = dir(foldername);

id = [];
imgfiles = {};
for i = 1:length(files)
    [~,name,ext] = fileparts(files(i).name);
    if strcmp(ext,'.bmp') == 1
        id(end+1) = str2num(name);
        imgfiles(end+1) = {fullfile(files(i).folder,strcat(name,ext))};
    end
end

[nameid,id] = sort(id,'ascend');
imgfiles = imgfiles(id);

if ~exist(fullfile(foldername,'cameraR.mat'),'file')
    patternSize = [7,5];
    squareSize = 0.112;
    [X,Y] = ndgrid(0:patternSize(1)-1, 0:patternSize(2)-1);
    pts_o = [X(:)*squareSize Y(:)*squareSize zeros(prod(patternSize),1)];  % Z=0
    %%
    files = dir(fullfile(foldername,'*.yml'));
    latest = [];
    for i = length(files)
        file = dir(fullfile(files(i).folder,files(i).name));
        if isempty(latest)
            latest.id = i;
            latest.date = datenum(file.date);
        else
            if datenum(file.date) > latest.date
                latest.id = i;
                latest.date = datenum(file.date);
            end
        end
    end
    califile = fullfile(files(latest.id).folder,files(latest.id).name);
%     califile = fullfile(foldername,'cali_param_12_23_14_12.yml');
    fs=cv.FileStorage(califile);
    CAMERA_MATRIX = fs.Camera_Matrix;
    CAMERA_DIST = fs.Distortion_Coefficients;
    disp(CAMERA_MATRIX);
    fprintf('\n');
    disp(CAMERA_DIST);
    options = {'WinSize',[11 11], 'Criteria',...
        struct('type','Count+EPS', 'maxCount',30, 'epsilon',0.01)};
    Rs = zeros(3,3,length(imgfiles));
    for i = 1:length(imgfiles)
        im = cv.imread(imgfiles{i}, 'Color',true);
        im_l = cv.imread(imgfiles{i}, 'Grayscale',true);
        pts_l = cv.findChessboardCorners(im_l, patternSize);
        pts_l = cv.cornerSubPix(im_l, pts_l, options{:});
        im_l = cv.drawChessboardCorners(im, patternSize, pts_l);

        imagepoints = reshape(cell2mat(pts_l),2,[])';
        % run pnp
        [rvec, tvec, success, inliers] = cv.solvePnPRansac(pts_o, imagepoints, CAMERA_MATRIX, 'DistCoeffs', CAMERA_DIST);
        % reproj
        R = expSO3(rvec);% this R is from world to camera
        points = (R * pts_o' + repmat(tvec,1,length(pts_o)))';
        repro_points = cv.projectPoints(points, [0,0,0], [0,0,0], CAMERA_MATRIX, 'DistCoeffs', CAMERA_DIST);
        imshow(im_l);hold on;
        plot(repro_points(:,1),repro_points(:,2),'gs');
        drawnow
        hold off;
        pause(0.1);
        Rs(:,:,i) = R;
    end
    save(fullfile(foldername,'cameraR.mat'),'Rs');
else
    load(fullfile(foldername,'cameraR.mat'));
end

if ismac
    symb = '/';
else
    symb = '/';
end

if 1%~exist(fullfile(foldername,'gyroR.mat'),'file')
    tmp = split(foldername,symb);
    name = tmp{end};
    % gpsfile = fullfile(foldername,strcat(name,'-gps.csv'));
    gyrofile = fullfile(foldername,strcat(name,'-gyro.csv'));
    % accfile = fullfile(foldername,strcat(name,'-accl.csv'));
    % gopro_gps = csvread(gpsfile,1);
    gopro_gyro = csvread(gyrofile,1)';
    % gopro_acc = csvread(accfile,1);
    framerate = 30;
    dt = mean(diff(gopro_gyro(1,:)))*1e-3;            
    % integration to get the rotation, this rotation is the rotation from initial frame to current frame
    q = integrate_gyro_quaternion_uniform(gopro_gyro(2:4,:), dt, []);

    matchedq = zeros(4,length(imgfiles));
    for i = 1:length(imgfiles)
        frametime = nameid(i) / 30 * 1e3;% s to ms
        [minval, minid] = min(abs(gopro_gyro(1,:)-frametime));
        ii = []; jj = [];
        if gopro_gyro(1,minid) > frametime && minid > 1
            ii = minid - 1;
            jj = minid;
        elseif gopro_gyro(1,minid) > frametime && minid < length(gopro_gyro)
            ii = minid;
            jj = minid + 1;
        end
        % slerp interpolation
        if ~isempty(ii) && ~isempty(jj) 
            q1 = slerp(q(:,ii), q(:,jj), (frametime-gopro_gyro(1,ii))/(gopro_gyro(1,jj)-gopro_gyro(1,ii)));
        else
            q1 = q(:,minid);
        end
        if minval < 8
            matchedq(:,i) = q1;
        end
    end
    save(fullfile(foldername,'gyroR.mat'),'matchedq');
else
    load(fullfile(foldername,'gyroR.mat'));
end
videoaxes=[];
gyroaxes=[];
anglediffs=[];
id = [];
for i = 2:size(Rs,3)
    Rc1 = Rs(:,:,i-1);
    Rc2 = Rs(:,:,i);
    
    Rg1 = quat_to_rotation_matrix(matchedq(:,i-1));
    Rg2 = quat_to_rotation_matrix(matchedq(:,i));
    
    dRc = Rc2 * Rc1';
    dRg = Rg2' * Rg1;
    
    [v1, theta1] = rotation_matrix_to_axis_angle(dRc);
    [v2, theta2] = rotation_matrix_to_axis_angle(dRg);
%     [v3, theta3] = rotation_matrix_to_axis_angle(Rg2 * Rg1');
    
    if theta1 < 0
        v1 = -v1;
        theta1 = -theta1;
    end
    
    if theta2 < 0
        v2 = -v2;
        theta2 = -theta2;
    end
    
    anglediff = abs(theta2 - theta1);
    if anglediff > pi
        anglediff = 2*pi - anglediff;
    end
    anglediffs(end+1) = anglediff;
    if anglediff < 8 * pi /180.0
        videoaxes(:,end+1) = v1;
        gyroaxes(:,end+1) = v2;
        id(end+1)=i-1;
    end
end
aa = 1:length(anglediffs);
plot(aa, anglediffs);hold on;
plot(aa(id), anglediffs(id),'ro');

if length(gyroaxes) < 2
    error('Hand-eye calibration requires >= 2 rotations');
end

% [0] since we only cares about rotation
model_func = @(d) (procrustes(d(1:3,:), d(4:6,:), false));

% select to not use minimal case
model_points = 2; 
threshold = 8.0*pi/180;% tunable

data = [videoaxes;gyroaxes];
R1 = adaRANSAC(model_func, @eval_func, data, model_points, threshold, true);
[n, theta] = rotation_matrix_to_axis_angle(R1);
save(fullfile(foldername,'R1.mat'),'R1');

function theta=eval_func(model, d)
    X=d(1:3,:);
    Y=d(4:6,:);
    Rest = model;
    Xhat = Rest*Y;
    costheta = dot(Xhat,X);
    theta = acos(costheta);
end 