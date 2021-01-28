clc;close all;clear all;
addpath ../misc/
addpath utils\
% folder = '/Volumes/document/fore-end/data/20191219/3';%uigetdir();%'D:\dtu\data\hand_eye\1016\3';
folder = 'D:/dtu/data/hand_eye/20191219/3';

% traverse this folder to find the video name
lists = dir(folder);
for i = 1:length(lists)
    if lists(i).isdir
        if strfind(lists(i).name,'GH')
            videoname = lists(i).name;
            break;
        end
    end
end

% we have the vide name, we find some calibration information.
califile = fullfile(folder,videoname,'tmp','extrinsics.mat');
timeoffsetfile = fullfile(folder,videoname,'tmp','time_offset.mat');

% we find imugps data
GYRO_RATE_GUESS = 200;
imugps = GPS_IMU_Stream(GYRO_RATE_GUESS);
imufile = fullfile(folder,'imugps.mat');
imugps.from_mat(imufile, 5);

% we get calibration file and time offset file.
load(califile);
load(timeoffsetfile,'time_offset_to_imu_local');

% vision position
visionfile = fullfile(folder,'tmp','position-raw-rec.mat');
vdata = load(visionfile);

% fusion position, fusion-position-raw-rec or full-state-fusion-res-position-raw-rec
fusionfile = fullfile(folder,'tmp','full-state-fusion-res-position-raw-rec.mat');
fdata = load(fusionfile);

[pos1, time1, pos2, time2] = downsampleFusionData(vdata, fdata, time_offset_to_imu_local);

% we find imugps data on bird
GYRO_RATE_GUESS = 200;
bird_imugps = GPS_IMU_Stream(GYRO_RATE_GUESS);
imufile = fullfile(folder,'imugps_bird.mat');
bird_imugps.from_mat(imufile, 0);

% draw vision failure
rawvdata = load(fullfile(folder,'tmp\position-raw.txt'));
visionfail1 = find(rawvdata(:,3)==-1);
failtime1 = rawvdata(visionfail1) / 30.0 + time_offset_to_imu_local;
[minval,minid] = min(abs(bird_imugps.time-failtime1(1)));
if minval < 0.01
    time_offset_to_gps_time = bird_imugps.time(minid) - failtime1(1);
    failtime1 = failtime1 + time_offset_to_gps_time;
end
visionfail2 = find(rawvdata(:,9)==-1);
failtime2 = rawvdata(visionfail2) / 30.0 + time_offset_to_imu_local;
[minval,minid] = min(abs(bird_imugps.time-failtime2(1)));
if minval < 0.01
    time_offset_to_gps_time = bird_imugps.time(minid) - failtime2(1);
    failtime2 = failtime2 + time_offset_to_gps_time;
end

% convert to global position
[gtime1, Rs1, ts1, marker_pos1_in_imu] = find_time_R_t_bodypose(imugps, time1, pos1, time_offset_to_imu_local, R1.R1, R2, [-0.035;0.115;-0.080],1);
[gtime2, Rs2, ts2, marker_pos2_in_imu] = find_time_R_t_bodypose(imugps, time2, pos2, time_offset_to_imu_local, R1.R1, R2, [-0.035;0.115;-0.080],1);

% find matches 
[bird_time1, bird_pos1, bird_local_time1] = find_matches(gtime1,bird_imugps);
[bird_time2, bird_pos2, bird_local_time2] = find_matches(gtime2,bird_imugps);

% align them
[g_pos1_marker_refine] = find_relative_transformation_new(Rs1, ts1, marker_pos1_in_imu, bird_pos1);
[g_pos2_marker_refine] = find_relative_transformation_new(Rs2, ts2, marker_pos2_in_imu, bird_pos2);

g_pos1_marker_refine = EKF(g_pos1_marker_refine, gtime1, bird_pos1);
g_pos2_marker_refine = EKF(g_pos2_marker_refine, gtime2, bird_pos2);

% find inliers
[Ropt1,topt1,final_consensus1] = estimate_rigid_body_transformation_RANSAC(g_pos1_marker_refine, bird_pos1,0.2);
[Ropt2,topt2,final_consensus2] = estimate_rigid_body_transformation_RANSAC(g_pos2_marker_refine, bird_pos2,0.2);
g_pos1_marker_refine = Ropt1 * g_pos1_marker_refine + topt1;
g_pos2_marker_refine = Ropt2 * g_pos2_marker_refine + topt2;

% plot
figure(1);
subplot(3,1,1);
plot(bird_time1, bird_pos1(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime1, g_pos1_marker_refine(1,:), 'b-', 'LineWidth', 2);
legend({'SPAN','Vision'},'FontName','Arial','FontSize',15);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);
grid minor;

subplot(3,1,2);
plot(bird_time1, bird_pos1(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime1, g_pos1_marker_refine(2,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);
grid minor;

subplot(3,1,3);
plot(bird_time1, bird_pos1(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime1, g_pos1_marker_refine(3,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);
grid minor;

%print(fullfile(outfolder,'marker1'),'-dpng','-r300');

figure(2);
subplot(3,1,1);
plot(bird_time2, bird_pos2(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime2, g_pos2_marker_refine(1,:), 'b-', 'LineWidth', 2);
legend({'SPAN','Vision'},'FontName','Arial','FontSize',15);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);
grid minor;

subplot(3,1,2);
plot(bird_time2, bird_pos2(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime2, g_pos2_marker_refine(2,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);
grid minor;

subplot(3,1,3);
plot(bird_time2, bird_pos2(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime2, g_pos2_marker_refine(3,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);
grid minor;

%print(fullfile(outfolder,'marker2'),'-dpng','-r300');

% v1 = mean(abs(g_pos1_marker_refine(:,:)-bird_pos1(:,:)),2);
% s1 = std(abs(g_pos1_marker_refine(:,:)-bird_pos1(:,:)),0,2);
% 
% v2 = mean(abs(g_pos2_marker_refine(:,:)-bird_pos2(:,:)),2);
% s2 = std(abs(g_pos2_marker_refine(:,:)-bird_pos2(:,:)),0,2);
% 
% disp(v1);
% disp(s1);
% disp(v2);
% disp(s2);

v1 = mean([abs(g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1)), abs(g_pos2_marker_refine(:,final_consensus2)-bird_pos2(:,final_consensus2))],2);
s1 = std([abs(g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1)), abs(g_pos2_marker_refine(:,final_consensus2)-bird_pos2(:,final_consensus2))],0,2);
disp(v1);
disp(s1);

v2 = mean([abs(g_pos1_marker_refine(:,:)-bird_pos1(:,:)), abs(g_pos2_marker_refine(:,:)-bird_pos2(:,:))],2);
s2 = std([abs(g_pos1_marker_refine(:,:)-bird_pos1(:,:)), abs(g_pos2_marker_refine(:,:)-bird_pos2(:,:))],0,2);
disp(v2);
disp(s2);

err1 = (g_pos1_marker_refine(:,:)-bird_pos1(:,:));
err2 = (g_pos2_marker_refine(:,:)-bird_pos2(:,:));

bird_time1 = bird_local_time1(:);
bird_time2 = bird_local_time2(:);

figure(3);
subplot(3,1,1);
plot(bird_time1, err1(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
% title('Local-Level','FontName','Arial','FontSize',15);
% xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
ylim([-0.1,0.1]);
maxylim = max(ylim);
minylim = min(ylim);
% for i = 2:length(bird_time1)
%     if bird_time1(i) - bird_time1(i-1) > (1/30*3)
%         patch([bird_time1(i-1) bird_time1(i) bird_time1(i) bird_time1(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
%     end
% end
for i = 1:length(visionfail1)
    patch([failtime1(i) failtime1(i)+15/30 failtime1(i)+15/30 failtime1(i)], ...
        [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
end

subplot(3,1,2);
plot(bird_time1, err1(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
% xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
ylim([-0.1,0.1]);
% for i = 2:length(bird_time1)
%     if bird_time1(i) - bird_time1(i-1) > (1/30*3)
%         patch([bird_time1(i-1) bird_time1(i) bird_time1(i) bird_time1(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
%     end
% end
for i = 1:length(visionfail1)
    patch([failtime1(i) failtime1(i)+15/30 failtime1(i)+15/30 failtime1(i)], ...
        [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
end

subplot(3,1,3);
plot(bird_time1, err1(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
ylim([-0.1,0.1]);
% for i = 2:length(bird_time1)
%     if bird_time1(i) - bird_time1(i-1) > (1/30*3)
%         patch([bird_time1(i-1) bird_time1(i) bird_time1(i) bird_time1(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
%     end
% end
for i = 1:length(visionfail1)
    patch([failtime1(i) failtime1(i)+15/30 failtime1(i)+15/30 failtime1(i)], ...
        [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
end
%print(fullfile(outfolder,'err1'),'-dpng','-r300');

figure(4);
subplot(3,1,1);
plot(bird_time2, err2(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
% title('Local-Level','FontName','Arial','FontSize',15);
% xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
ylim([-0.1,0.1]);
% for i = 2:length(bird_time2)
%     if bird_time2(i) - bird_time2(i-1) > (1/30*3)
%         patch([bird_time2(i-1) bird_time2(i) bird_time2(i) bird_time2(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
%     end
% end
for i = 1:length(visionfail2)
    patch([failtime2(i) failtime2(i)+15/30 failtime2(i)+15/30 failtime2(i)], ...
        [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
end

subplot(3,1,2);
plot(bird_time2, err2(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
% xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
ylim([-0.1,0.1]);
% for i = 2:length(bird_time2)
%     if bird_time2(i) - bird_time2(i-1) > (1/30*3)
%         patch([bird_time2(i-1) bird_time2(i) bird_time2(i) bird_time2(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
%     end
% end
for i = 1:length(visionfail2)
    patch([failtime2(i) failtime2(i)+15/30 failtime2(i)+15/30 failtime2(i)], ...
        [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
end

subplot(3,1,3);
plot(bird_time2, err2(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
ylim([-0.1,0.1]);
% for i = 2:length(bird_time2)
%     if bird_time2(i) - bird_time2(i-1) > (1/30*3)
%         patch([bird_time2(i-1) bird_time2(i) bird_time2(i) bird_time2(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
%     end
% end
for i = 1:length(visionfail2)
    patch([failtime2(i) failtime2(i)+15/30 failtime2(i)+15/30 failtime2(i)], ...
        [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
end
%print(fullfile(outfolder,'err2'),'-dpng','-r300');


%% generate google map view
E = wgs84Ellipsoid;
lat0 = 55+46/60+59.62892/3600;
long0 = 12+30/60+57.55967/3600;
h0 = 97.717;
[lat_bird,lon_bird,h_bird] = ned2geodetic(bird_pos2(2,:),bird_pos2(1,:),-bird_pos2(3,:),lat0,long0,h0,E);
[lat_marker,lon_marker,h_marker] = ned2geodetic(g_pos2_marker_refine(2,:),g_pos2_marker_refine(1,:),-g_pos2_marker_refine(3,:),lat0,long0,h0,E);

% plot route data
figure
plot(lon_bird, lat_bird, 'r--', 'LineWidth', 2);hold on;
plot(lon_marker, lat_marker, 'g-.', 'LineWidth', 2);
xpos = xlim;%[min(lon_bird), max(lon_bird)];
ypos = ylim;%[min(lat_bird), max(lat_bird)];
xlim([xpos(1)-0.00001, xpos(2)+0.00001])
ylim([ypos(1)-0.00001, ypos(2)+0.00001])
legend({'Ref','Est'},'FontSize',10,'FontName','Arial');
% title('Map View');
xlabel('Longitude: minutes ('') seconds ('''') ');
ylabel('Latitude: minutes ('') seconds ('''') ');

% Google map
addpath 'D:\dtu\sourcecode\plot_google_map\plot_google_map-master'
plot_google_map('maptype', 'satellite');

%% ticks to minites
xtks = xticks - 12;
ytks = yticks - 55;
ytkmins = fix(ytks * 60);
xtkmins = fix(xtks * 60);
ytksecs = round((ytks * 60 - ytkmins) * 60);
xtksecs = round((xtks * 60 - xtkmins) * 60);

xtklbs = cell(length(xtks),1);
for i = 1:2:length(xtks)
    xtklbs{i} = sprintf('%2d''%2d''''',xtkmins(i),xtksecs(i));
end

ytklbs = cell(length(ytks),1);
for i = 1:length(ytks)
    ytklbs{i} = sprintf('%2d''%2d''''',ytkmins(i),ytksecs(i));
end

xticklabels(xtklbs);
yticklabels(ytklbs);

%print(fullfile(outfolder,'map'),'-dpng','-r300');

rmpath ../misc/
rmpath utils\

function pos = EKF(pos0, time, y)
    x = [pos0(:,1);[0;0;0]];
    P = blkdiag(eye(3),eye(3)*1000);
    R1 = eye(3)*0.1;
%     R2 = diag([0.04 0.04 0.01]);%eye(3)*0.03;%0.009 2-3 type1
    
%     R2 = diag([0.5 0.5 0.04]);%1
    R2 = diag([0.2 0.2 0.02]);%eye(3)*0.03;%0.009 2-3 type2
    Q = eye(6)*1;
    for i = 1:length(pos0)
        C = [eye(3) zeros(3);eye(3) zeros(3)];
        K = P*C'*inv(C*P*C' + blkdiag(R1,R2));
        x = x + K*([pos0(:,i);y(:,i)]-C*x);
        P = (eye(6)-K*C)*P;
        pos(:,i)=x(1:3);
        
        if i > 1
            dt = time(i) - time(i-1);
            A = [eye(3) eye(3)*dt;zeros(3) eye(3)];
            x = A*x;
            P = A*P*A' + Q;
        end
    end
end

function [pos1, time1, pos2, time2] = downsampleFusionData(vdata, fdata, time_offset_to_imu_local)
    pos1 = vdata.pos1;
    for i = 1:length(vdata.time1)
        align_time = vdata.time1(i,2) + time_offset_to_imu_local;
        [minval,minid] = min(abs(fdata.fusion_time1-(align_time)));
        if minval < 0.01
            pos1(i,2:4) = fdata.pos_filtered1(:,minid)';
        end
    end
    time1 = vdata.time1;
    
    pos2 = vdata.pos2;
    for i = 1:length(vdata.time2)
        align_time = vdata.time2(i,2) + time_offset_to_imu_local;
        [minval,minid] = min(abs(fdata.fusion_time2-(align_time)));
        if minval < 0.01
            pos2(i,2:4) = fdata.pos_filtered2(:,minid)';
        end
    end
    time2 = vdata.time2;
end


