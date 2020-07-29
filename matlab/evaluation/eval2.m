clc;close all;clear all;
folder = '/Volumes/document/fore-end/data/20191219/3';%uigetdir();%'D:\dtu\data\hand_eye\1016\3';
lists = dir(folder);
for i = 1:length(lists)
    if lists(i).isdir
        if strfind(lists(i).name,'GH')
            videoname = lists(i).name;
        end
    end
end

califile = fullfile(folder,videoname,'tmp','extrinsics.mat');
timeoffsetfile = fullfile(folder,videoname,'tmp','time_offset.mat');
GYRO_RATE_GUESS = 200;
imugps = GPS_IMU_Stream(GYRO_RATE_GUESS);
imufile = fullfile(folder,'imugps.mat');
imugps.from_mat(imufile, 5);

load(califile);
load(timeoffsetfile,'time_offset_to_imu_local');

% vision position
visionfile = fullfile(folder,'tmp','position_rec.mat');
load(visionfile);

GYRO_RATE_GUESS = 200;
bird_imugps = GPS_IMU_Stream(GYRO_RATE_GUESS);
imufile = fullfile(folder,'imugps_bird.mat');
bird_imugps.from_mat(imufile, 0);


% convert to global
[gtime1,g_pos1_marker,marker_pos1_in_imu] = compose_g_pos_marker(imugps, time1, pos1, time_offset_to_imu_local, R1.R1, R2, [0;0;0]);
[gtime2,g_pos2_marker,marker_pos2_in_imu] = compose_g_pos_marker(imugps, time2, pos2, time_offset_to_imu_local, R1.R1, R2, [0;0;0]);

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
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);

subplot(3,1,2);
plot(bird_time1, bird_pos1(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime1, g_pos1_marker_refine(2,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);

subplot(3,1,3);
plot(bird_time1, bird_pos1(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime1, g_pos1_marker_refine(3,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);

figure(2);
subplot(3,1,1);
plot(bird_time2, bird_pos2(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime2, g_pos2_marker_refine(1,:), 'b-', 'LineWidth', 2);
legend({'SPAN','Vision'},'FontName','Arial','FontSize',15);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);

subplot(3,1,2);
plot(bird_time2, bird_pos2(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime2, g_pos2_marker_refine(2,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);

subplot(3,1,3);
plot(bird_time2, bird_pos2(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
plot(gtime2, g_pos2_marker_refine(3,:), 'b-', 'LineWidth', 2);
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);



v1 = mean(vecnorm(g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1)));
s1 = std(vecnorm(g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1)));

v2 = mean(vecnorm(g_pos2_marker_refine(:,final_consensus2)-bird_pos2(:,final_consensus2)));
s2 = std(vecnorm(g_pos1_marker_refine(:,final_consensus1)-bird_pos1(:,final_consensus1)));

disp(v1);
disp(v2);

err1 = (g_pos1_marker_refine(:,:)-bird_pos1(:,:));
err2 = (g_pos2_marker_refine(:,:)-bird_pos2(:,:));

% inlierid1 = abs(err1 - v1) <= s1*3;
% inlierid2 = abs(err2 - v2) <= s2*3;

% bird_time1 = bird_time1(final_consensus1);
% err1 = err1(:,final_consensus1);
% err2 = err2(:,final_consensus2);

figure(3);
subplot(3,1,1);
plot(bird_time1, err1(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);

subplot(3,1,2);
plot(bird_time1, err1(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);

subplot(3,1,3);
plot(bird_time1, err1(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);

figure(4);
subplot(3,1,1);
plot(bird_time2, err2(1,:), 'r-', 'LineWidth', 2); hold on;grid on;
title('Local-Level','FontName','Arial','FontSize',15);
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('x: (m)','FontName','Arial','FontSize',15);

subplot(3,1,2);
plot(bird_time2, err2(2,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('y: (m)','FontName','Arial','FontSize',15);

subplot(3,1,3);
plot(bird_time2, err2(3,:), 'r-', 'LineWidth', 2); hold on;grid on;
xlabel('time: (sec)','FontName','Arial','FontSize',15);
ylabel('z: (m)','FontName','Arial','FontSize',15);

function pos = EKF(pos0, time, y)
    x = [pos0(:,1);[0;0;0]];
    P = blkdiag(eye(3),eye(3)*1000);
    R1 = eye(3)*1;
    R2 = eye(3)*0.5;
    Q = eye(6)*0.1;
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

function [R,t,final_consensus] = estimate_rigid_body_transformation_RANSAC(ptsrc, ptdst)
    data = [ptsrc;ptdst];
    threshold = 0.1;
    recalculate = true;
    model_func = @estimate_rigid_body_transformation_SVD;
    num_points = 3;
    
    function err = eval_func(model, data)
        R = model.Ropt;
        t = model.topt;
        x = data(1:3,:);
        y = data(4:6,:);
        yhat = R*x+t;
        err = y-yhat;
        err = vecnorm(err);
    end
    
    [M,final_consensus] = adaRANSAC(model_func, @eval_func, data, num_points, threshold, recalculate);
    R = M.Ropt;
    t = M.topt;
end

function align_pos_g = estimate_align_pos_g(Rs, body_pos_g, align_pos_body)
    align_pos_g = zeros(size(align_pos_body));
    for i = 1:size(Rs,3)
        align_pos_g(:,i) = Rs(:,:,i)*align_pos_body(:,i) + body_pos_g(:,i);
    end
end

function [t, bird_pos_in_body, body_pos_g, Rs] = estimate_bird_pos(imugps, bird_imugps,gtime)
    t = zeros(1, length(gtime));
    bird_pos_in_body = zeros(3, length(gtime));
    for i = 1:length(gtime)
        [minval1,minid1] = min(abs(bird_imugps.GPSTime-gtime(i)));
        [minval2,minid2] = min(abs(imugps.GPSTime-gtime(i)));
        if (minval1 - minval2) < 0.008
            rpy = [imugps.yaw(minid2);imugps.pitch(minid2);imugps.roll(minid2)];
            R = angle2dcm(rpy(1),rpy(2),rpy(3));
            t(i) = imugps.GPSTime(minid2);
            bird_pos_in_body(:,i) = R'*(bird_imugps.p(:,minid1) - imugps.p(:,minid2)); 
            body_pos_g(:,i) = imugps.p(:,minid2);
            Rs(:,:,i) = R;
        end
    end
end

function varargout = estimate_rigid_body_transformation_SVD(varargin)
%% SVD 
if nargin == 1
    ptsrc = varargin{1}(1:3,:);
    ptdst = varargin{1}(4:6,:);
else
    ptsrc = varargin{1};
    ptdst = varargin{2};
end

ptsrcmean = mean(ptsrc,2);
ptdstmean = mean(ptdst,2);

ptsrcrefine = ptsrc - repmat(ptsrcmean, 1, size(ptsrc,2));
ptdstrefine = ptdst - repmat(ptdstmean, 1, size(ptsrc,2));

Y = ptdstrefine';
X = ptsrcrefine;
S = X*Y;
[U,~,V] = svd(S);

D = V*U';
if det(D) < 0
    Ropt = V*[1 0 0;0 1 0;0 0 -1]*U';
else
    Ropt = V*U';
end
topt = ptdstmean - Ropt * ptsrcmean;

if nargout == 2
    varargout{1} = Ropt;
    varargout{2} = topt;
else
    res.Ropt = Ropt;
    res.topt = topt;
    varargout{1} = res;
end

end

function [bird_time, bird_pos] = find_matches(gtime,bird_imugps)
    bird_time = zeros(1, length(gtime));
    bird_pos = zeros(3, length(gtime));
    for i = 1:length(gtime)
        [minval,minid] = min(abs(bird_imugps.GPSTime-gtime(i)));
        if minval < 0.01
            bird_time(i) = bird_imugps.GPSTime(minid);
            bird_pos(:,i) = bird_imugps.p(:,minid);
        else
            warning('data missing');
        end
    end
end



% R1: from the calibration programme is the rotation from imu1 to camera
% R2: from the calibration programme is the rotation from imu2 to imu1
% t2: from the calibration programme is the translation from imu2 to imu1
function [gtime, g_pos_marker, marker_pos_in_imu] = compose_g_pos_marker(imugps, time, pos, timeoffset, R1, R2, t2)
    marker_pos_in_camera = pos(:,2:4)';%3xN
    % ideally, there should be a t1 
    marker_pos_in_imu = R2' * (R1' * marker_pos_in_camera ) - t2;
    g_pos_marker = zeros(size(marker_pos_in_imu));
    gtime = zeros(1, length(time));
    for i = 1:length(time)
        t = time(i,2) + timeoffset;
        [minval,minid] = min(abs(imugps.time-(t)));
        if minval < 0.01
            uavpos = imugps.p(:,minid);%3x1
            rpy = [imugps.yaw(minid);imugps.pitch(minid);imugps.roll(minid)];
            R = angle2dcm(rpy(1),rpy(2),rpy(3));
            g_pos_marker(:,i) = R * marker_pos_in_imu(:,i) + uavpos;
            gtime(i) = imugps.GPSTime(minid);
        else
            warning('data missing');
        end
    end
end