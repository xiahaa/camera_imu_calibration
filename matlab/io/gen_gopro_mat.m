function gen_gopro_mat(videofile)
    [dir,name,~]=fileparts(videofile);
    if ~exist(fullfile(dir,'gopro_gps.mat'),'file') || ~exist(fullfile(dir,'gopro_imu.mat'),'file')
        gpsfile = fullfile(dir,strcat(name,'-gps.csv'));
        gyrofile = fullfile(dir,strcat(name,'-gyro.csv'));
        accfile = fullfile(dir,strcat(name,'-accl.csv'));
        gopro_gps = csvread(gpsfile,1);
        gopro_gyro = csvread(gyrofile,1);
        gopro_acc = csvread(accfile,1);
        gopro_imu = [gopro_acc gopro_gyro];
        save(fullfile(dir,'gopro_gps.mat'),'gopro_gps');
        save(fullfile(dir,'gopro_imu.mat'),'gopro_imu');
    end
end