close all;
%% first of all add path
if ismac
    addpath(genpath('/Volumes/document/camera_imu_calibration/matlab/'));
else
    % Change the current folder to the folder of this m-file.
    if(~isdeployed)
      cd(fileparts(which(mfilename)));
    end
    addpath(genpath('./'));
end


%% 
params = configParams();

fprintf('opening: \n\t %s \n\t %s \n\t %s',params.videofile, params.imufile, params.califile);
[filepath,name,ext] = fileparts(params.videofile);

%% load imu gps data
imugps = GPS_IMU_Stream(params.GYRO_RATE_GUESS);
imugps.from_mat(params.imufile, params.init_imu_t);

%% load camera calibration file
fs=cv.FileStorage(params.califile);
CAMERA_MATRIX = fs.Camera_Matrix;
CAMERA_DIST = fs.Distortion_Coefficients;
disp(CAMERA_MATRIX);
fprintf('\n');
disp(CAMERA_DIST);

%% load video
camera = OpenCVCameraModel(params.CAMERA_IMAGE_SIZE, params.CAMERA_FRAME_RATE, params.CAMERA_READOUT, CAMERA_MATRIX, CAMERA_DIST);
fprintf('Creating video stream from %s',params.videofile);
video=VideoStream(camera, 'optical');
try
    video.from_file(params.videofile,params.init_cam_t,params.duration_cam_t,fullfile(filepath,'gopro_imu'),fullfile(filepath,'gopro_gps'));
catch
    video.from_file(params.videofile,params.init_cam_t,inf);
end
% video.play();

%% seq-2:60,70,imu:185
%% seq-3:10,70,imu:135
%% seq-1:145,120      imu:100

save_path = fullfile(filepath,name,'tmp');
calib = CalibrationDTU(video,imugps,5,save_path);
% % try
    calib.initialize(params.GYRO_RATE_GUESS,params.Masterdata,params.t2);
% % catch
% %     error('calibration failure');
% % end
% 
% calib.calibrate();
% 
% 
% % Compare with reference data
% reference_data = csvread(referencefile,1);
% reference_data([3,4,5,6,7,8]) = reference_data([6,7,8,3,4,5]); % Swap order of bias and rot
% disp('\nCompare with reference data')
% fprintf('{:%s} {:%s} {:%s} {:%s}','Parameter', 'Reference', 'Optimized', 'Difference\n');
% name={'gyrorate','offset','bias_x','bias_y','bias_z','rot_x','rot_y','rot_z\n'};
% for i = 1:length(name)
%     fprintf('%s: %f, %f, %f\n', name{i}, reference_data(i), calib.result(i), reference_data(i)-calib.result(i));
% end
% 
% R_ref = to_rot_matrix(reference_data(6:end));
% R_data = to_rot_matrix(calib.result(6:end));
% dR = R_ref'*R_data;
% [v, theta] = rotation_matrix_to_axis_angle(dR);
% fprintf('Reference rotation\n');
% disp(R_ref);
% fprintf('Optimized rotation\n');
% disp(R_data);
% fprintf("Angle difference: %f degrees\n", theta*180/pi);


