clc;close all;clear all;
addpath ../misc/
addpath utils\

[folder, outfolder] = configdata();
videofile = dir(fullfile(folder,'*.MP4'));
[~,filename,fileext]=fileparts(videofile.name);
califile = fullfile(folder,strcat(filename,'\tmp\','extrinsics.mat'));
timeoffsetfile = fullfile(folder,strcat(filename,'\tmp\','time_offset.mat'));
GYRO_RATE_GUESS = 200;
imugps = GPS_IMU_Stream(GYRO_RATE_GUESS);
imufile = fullfile(folder,'imugps.mat');
imugps.from_mat(imufile, 5);

load(califile);
load(timeoffsetfile,'time_offset_to_imu_local');

% vision position
visionfile = fullfile(folder,strcat(filename,'\img\','position_rec.mat'));
load(visionfile);

GYRO_RATE_GUESS = 200;
bird_imugps = GPS_IMU_Stream(GYRO_RATE_GUESS);
imufile = fullfile(folder,'imugps_bird.mat');
bird_imugps.from_mat(imufile, 0);


% convert to global
[gtime1,g_pos1_marker,marker_pos1_in_imu] = compose_g_pos_marker(imugps, time1, pos1, time_offset_to_imu_local, R1.R1, R2, [-0.035;0.115;-0.080]);
[gtime2,g_pos2_marker,marker_pos2_in_imu] = compose_g_pos_marker(imugps, time2, pos2, time_offset_to_imu_local, R1.R1, R2, [-0.035;0.115;-0.080]);

[time1, bird_pos1_in_body, body_pos1_g, Rs1] = estimate_bird_pos(imugps, bird_imugps,gtime1);
[time2, bird_pos2_in_body, body_pos2_g, Rs2] = estimate_bird_pos(imugps, bird_imugps,gtime2);

[Ropt1,topt1,final_consensus1] = estimate_rigid_body_transformation_RANSAC(marker_pos1_in_imu, bird_pos1_in_body);
[Ropt2,topt2,final_consensus2] = estimate_rigid_body_transformation_RANSAC(marker_pos2_in_imu, bird_pos2_in_body);

% align them
g_pos1_marker_refine = Ropt1 * marker_pos1_in_imu + topt1;
g_pos2_marker_refine = Ropt2 * marker_pos2_in_imu + topt2;

g_pos1_marker_refine = estimate_align_pos_g(Rs1, body_pos1_g, g_pos1_marker_refine);
g_pos2_marker_refine = estimate_align_pos_g(Rs2, body_pos2_g, g_pos2_marker_refine);

% find matches
[bird_time1, bird_pos1] = find_matches(gtime1,bird_imugps);
[bird_time2, bird_pos2] = find_matches(gtime2,bird_imugps);

g_pos1_marker_refine = EKF(g_pos1_marker_refine, gtime1, bird_pos1);
g_pos2_marker_refine = EKF(g_pos2_marker_refine, gtime2, bird_pos2);

[Ropt1,topt1,final_consensus1] = estimate_rigid_body_transformation_RANSAC(g_pos1_marker_refine, bird_pos1);
[Ropt2,topt2,final_consensus2] = estimate_rigid_body_transformation_RANSAC(g_pos2_marker_refine, bird_pos2);
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

print(fullfile(outfolder,'marker1'),'-dpng','-r300');

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

print(fullfile(outfolder,'marker2'),'-dpng','-r300');

v1 = mean(abs(g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1)),2);
s1 = std(abs(g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1)),0,2);

v2 = mean(abs(g_pos2_marker_refine(:,final_consensus2)-bird_pos2(:,final_consensus2)),2);
s2 = std(abs(g_pos2_marker_refine(:,final_consensus2)-bird_pos2(:,final_consensus2)),0,2);

disp(v1);
disp(s1);
disp(v2);
disp(s2);

% err1 = zeros(3, size(g_pos1_marker_refine,2));
% err2 = zeros(3, size(g_pos2_marker_refine,2));
err1 = (g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1));
err2 = (g_pos2_marker_refine(:,final_consensus2)-bird_pos2(:,final_consensus2));

% inlierid1 = abs(err1 - v1) <= s1*3;
% inlierid2 = abs(err2 - v2) <= s2*3;

bird_time1 = bird_time1(final_consensus1);
bird_time2 = bird_time2(final_consensus2);

% err1 = err1(:,inlierid1);
% err2 = err2(:,inlierid2);

figure(3);
subplot(3,1,1);
plot(bird_time1, err1(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
% title('Local-Level','FontName','Arial','FontSize',15);
% xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
for i = 2:length(bird_time1)
    if bird_time1(i) - bird_time1(i-1) > (1/30*3)
        patch([bird_time1(i-1) bird_time1(i) bird_time1(i) bird_time1(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
    end
end

subplot(3,1,2);
plot(bird_time1, err1(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
% xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
for i = 2:length(bird_time1)
    if bird_time1(i) - bird_time1(i-1) > (1/30*3)
        patch([bird_time1(i-1) bird_time1(i) bird_time1(i) bird_time1(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
    end
end

subplot(3,1,3);
plot(bird_time1, err1(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
for i = 2:length(bird_time1)
    if bird_time1(i) - bird_time1(i-1) > (1/30*3)
        patch([bird_time1(i-1) bird_time1(i) bird_time1(i) bird_time1(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
    end
end
print(fullfile(outfolder,'err1'),'-dpng','-r300');

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
for i = 2:length(bird_time2)
    if bird_time2(i) - bird_time2(i-1) > (1/30*3)
        patch([bird_time2(i-1) bird_time2(i) bird_time2(i) bird_time2(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
    end
end

subplot(3,1,2);
plot(bird_time2, err2(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
% xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
for i = 2:length(bird_time2)
    if bird_time2(i) - bird_time2(i-1) > (1/30*3)
        patch([bird_time2(i-1) bird_time2(i) bird_time2(i) bird_time2(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
    end
end

subplot(3,1,3);
plot(bird_time2, err2(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (s)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);
grid minor;
axis tight
maxylim = max(ylim);
minylim = min(ylim);
for i = 2:length(bird_time2)
    if bird_time2(i) - bird_time2(i-1) > (1/30*3)
        patch([bird_time2(i-1) bird_time2(i) bird_time2(i) bird_time2(i-1)], [maxylim maxylim minylim minylim], [0.1 0.1 0.1], 'FaceAlpha',0.2,'EdgeColor','none');
    end
end
print(fullfile(outfolder,'err2'),'-dpng','-r300');


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

print(fullfile(outfolder,'map'),'-dpng','-r300');

rmpath ../misc/
rmpath utils\

function pos = EKF(pos0, time, y)
    x = [pos0(:,1);[0;0;0]];
    P = blkdiag(eye(3),eye(3)*1000);
    R1 = eye(3)*0.1;
    R2 = eye(3)*0.009;%0.009 2-3
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