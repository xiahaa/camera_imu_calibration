%% first of all add path
addpath(genpath('/Volumes/document/camera_imu_calibration/matlab/'));
%% constants 
CAMERA_MATRIX = [[ 853.12703455,    0.        ,  988.06311256]; ...
                 [   0.        ,  873.54956631,  525.71056312]; ...
                 [   0.        ,    0.        ,    1.        ]];
CAMERA_DIST_CENTER = [0.00291108,  0.00041897];
CAMERA_DIST_PARAM = 0.8894355;
CAMERA_FRAME_RATE = 30.0;
CAMERA_IMAGE_SIZE = [1920, 1080];
CAMERA_READOUT = 0.0316734;
GYRO_RATE_GUESS = 853.86;
%%
videofile = '../data/gopro-gyro-dataset/rccar.MP4';
[filepath,name,ext] = fileparts(videofile);
gyrofile = fullfile(filepath,strcat(name,'_gyro.csv'));
referencefile = fullfile(filepath,strcat(name,'_reference.csv'));
sprintf('opening: \n\t %s \n\t %s \n\t %s', videofile, gyrofile, referencefile);

%%
gyro = GyroStream();
disp(['Creating gyro stream from ',gyrofile]);
gyro.from_csv(gyrofile);
disp(['Post processing L3G4200D gyroscope data to remove frequency spike noise']);
gyro.prefilter(false);

%%
camera = AtanCameraModel(CAMERA_IMAGE_SIZE, CAMERA_FRAME_RATE, CAMERA_READOUT, CAMERA_MATRIX,CAMERA_DIST_CENTER, CAMERA_DIST_PARAM);
sprintf('Creating video stream from %s',videofile);
video=VideoStream(camera, 'optical');
video.from_file(videofile);
% video.play();

%%
save_path = fullfile(filepath,name,'tmp');
calib = Calibration(video,gyro,5,save_path);
% try
    calib.initialize(GYRO_RATE_GUESS);
% catch
%     error('calibration failure');
% end

calib.calibrate();


% Compare with reference data
reference_data = csvread(referencefile,1);
reference_data([3,4,5,6,7,8]) = reference_data([6,7,8,3,4,5]); % Swap order of bias and rot
disp('\nCompare with reference data')
fprintf('{:%s} {:%s} {:%s} {:%s}','Parameter', 'Reference', 'Optimized', 'Difference\n');
name={'gyrorate','offset','bias_x','bias_y','bias_z','rot_x','rot_y','rot_z\n'};
for i = 1:length(name)
    fprintf('%s: %f, %f, %f\n', name{i}, reference_data(i), calib.result(i), reference_data(i)-calib.result(i));
end

R_ref = to_rot_matrix(reference_data(6:end));
R_data = to_rot_matrix(calib.result(6:end));
dR = R_ref'*R_data;
[v, theta] = rotation_matrix_to_axis_angle(dR);
fprintf('Reference rotation\n');
disp(R_ref);
fprintf('Optimized rotation\n');
disp(R_data);
fprintf("Angle difference: %f degrees\n", theta*180/pi);


