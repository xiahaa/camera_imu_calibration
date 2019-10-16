close all;
%% first of all add path
addpath(genpath('/Volumes/document/camera_imu_calibration/matlab/'));
%% constants 
GYRO_RATE_GUESS = 100;
CAMERA_FRAME_RATE = 30.0;
CAMERA_IMAGE_SIZE = [2704, 1520];
CAMERA_READOUT = 0.03;
%% 
videofile = '../data/calibration/seq1/GH013861.MP4';
imufile = '../data/calibration/seq1/imugps.mat';
califile = '../data/calibration/seq1/cali_param_1_9_16_53.yml';
fprintf('opening: \n\t %s \n\t %s \n\t %s',videofile, imufile, califile);
[filepath,name,ext] = fileparts(videofile);

%% load imu gps data
imugps = GPS_IMU_Stream();
imugps.from_mat(imufile, 100);

%% load camera calibration file
fs=cv.FileStorage(califile);
CAMERA_MATRIX = fs.Camera_Matrix;
CAMERA_DIST = fs.Distortion_Coefficients;
disp(CAMERA_MATRIX);
fprintf('\n');
disp(CAMERA_DIST);

%% load video
camera = OpenCVCameraModel(CAMERA_IMAGE_SIZE, CAMERA_FRAME_RATE, CAMERA_READOUT, CAMERA_MATRIX, CAMERA_DIST);
fprintf('Creating video stream from %s',videofile);
video=VideoStream(camera, 'optical');
video.from_file(videofile,145,120,fullfile(filepath,'gopro_imu'),fullfile(filepath,'gopro_gps'));
% video.play();

%% seq-2:60,70,imu:185
%% seq-3:10,70,imu:135
%% seq-1:145,120      imu:100

save_path = fullfile(filepath,name,'tmp');
calib = CalibrationDTU(video,imugps,5,save_path);
% % try
    calib.initialize(GYRO_RATE_GUESS);
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

