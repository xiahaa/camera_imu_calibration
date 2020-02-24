clc;close all;
addpath('../misc');
basename = configdata();
videofile = dir(fullfile(basename,'*.MP4'));
[~,name,ext] = fileparts(videofile.name);
foldername = fullfile(basename,name,'img');

if ~exist(fullfile(foldername,'position.mat'),'file')
    filename = fullfile(foldername,'position.log');
%     data = uiimport(filename);
    data = txtread(filename);
    save(fullfile(foldername,'position.mat'),'data');
else
    load(fullfile(foldername,'position.mat'));
end

% time: where you start computing, TODO, make this automatically
start_id = 600;
fid = fopen(fullfile(foldername,'time.txt'),'r');
time = fscanf(fid,'%f %f',[2,inf])';
% time = time(start_id+1:end,:);

pos = data;
N = length(pos);
Nm = round((size(pos,2)-1)/4);
%% make id consistent
% meas = zeros(N,4,Nm);
% id = 1;
% for i = 1:N
%     % find first valid line
%     if id == 1
%         if isempty(find(isnan(pos(i,2:end))))
%             for j = 1:Nm
%                 meas(id,:,j) = pos(i,[1,3+(j-1)*4,4+(j-1)*4,5+(j-1)*4]);
%             end
%             id = id + 1;
%         else
%             continue;
%         end
%     else
%         validlen = round(find(~isnan(pos(i,2:end)))/4);
%         for j = 1:validlen
%             currmeas = pos(i,[1,3+(j-1)*4,4+(j-1)*4,5+(j-1)*4]);        
%             
%         end
%         end
%     end
% end

id = find(sum(isnan(pos),2) == 0,1);
pos = pos(id:end,:);
time = time(pos(:,1)+start_id,:);
for i = 2:N
    pos1 = pos(i,[3,4,5]);
    pos2 = pos(i,[7,8,9]);
    
    prevpos1 = pos(i-1,[3,4,5]);
    prevpos2 = pos(i-1,[7,8,9]);
    
    swap = 0;
    
    if sum(isnan(pos1)) == 0
        % pos1 valid
        if sum(isnan(prevpos1)) == 0 && sum(isnan(prevpos2)) == 0
            % prevpos1 valid
            err1 = norm(pos1-prevpos1);
            err2 = norm(pos1-prevpos2);
            if err2 < err1
                % swap
                swap = 1;
            end
        elseif sum(isnan(prevpos2)) == 0
            err2 = norm(pos1-prevpos2);
            if err2 < 0.15
                % swap
                swap = 1;
            end
        end
    end
    
    if swap == 0
        if sum(isnan(pos2)) == 0
            % pos1 valid
            if sum(isnan(prevpos1)) == 0 && sum(isnan(prevpos2)) == 0
                % prevpos1 valid
                err1 = norm(pos2-prevpos1);
                err2 = norm(pos2-prevpos2);
                if err1 < err2
                    % swap
                    swap = 1;
                end
            elseif sum(isnan(prevpos1)) == 0
                err1 = norm(pos2-prevpos1);
                if err1 < 0.15
                    % swap
                    swap = 1;
                end
            end
        end
    end
    
    if swap == 1
        pos(i,[3,4,5]) = pos2;
        pos(i,[7,8,9]) = pos1;
    end
end
    
%     if sum(isnan(pos1)) == 0 && sum(isnan(prevpos1)) == 0 && sum(isnan(prevpos2)) == 0
%         err1 = norm(pos1-prevpos1);
%         err2 = norm(pos1-prevpos2);
%         if err2 < err1
%             % swap
%             pos(i,[3,4,5]) = pos2;
%             pos(i,[7,8,9]) = pos1;
%         end
%     elseif sum(isnan(prevpos1)) == 0 && sum(isnan(pos1)) == 0 && sum(isnan(pos2)) == 0
%         err1 = norm(pos1-prevpos1);
%         err2 = norm(pos2-prevpos1);
%         if err2 < err1
%             % swap
%             pos(i,[3,4,5]) = pos2;
%             pos(i,[7,8,9]) = pos1;
%         end
%     elseif 
%     end

pos1 = pos(:,[1,3,4,5]);
pos2 = pos(:,[1,7,8,9]);

% filter out nan
id = sum(isnan(pos1),2) == 0 & pos1(:,2) ~= -1;
pos1 = pos1(id,:);
time1 = time(id,:);

id = sum(isnan(pos2),2) == 0 & pos2(:,2) ~= -1;
pos2 = pos2(id,:);
time2 = time(id,:);

%% filtering, interpolation to make data consistent
hsize = 3;
posfit1 = pos1;
posfit2 = pos2;

N = length(pos1);
for i = 1:length(pos1)
    if i > hsize && i <= N-hsize
        id = i+(-hsize:1:hsize);
    elseif i < hsize
        id = 1:(2*hsize+1);
    else
        id = (N-2*hsize):N;
    end
    id1 = id(id ~= i);
    ypred(1) = interp1(id1,posfit1(id1,2),i);
    ypred(2) = interp1(id1,posfit1(id1,3),i);
    ypred(3) = interp1(id1,posfit1(id1,4),i);
    % 
    if norm(ypred-posfit1(i,2:4)) > 0.1
        posfit1(i,2:4) = ypred;
    end
end

N = length(pos2);
for i = 1:length(pos2)
    if i > hsize && i <= N-hsize
        id = i+(-hsize:1:hsize);
    elseif i < hsize
        id = 1:(2*hsize+1);
    else
        id = (N-2*hsize):N;
    end
    id1 = id(id ~= i);
    ypred(1) = interp1(id1,posfit2(id1,2),i);
    ypred(2) = interp1(id1,posfit2(id1,3),i);
    ypred(3) = interp1(id1,posfit2(id1,4),i);
    % 
    if norm(ypred-posfit2(i,2:4)) > 0.1
        posfit2(i,2:4) = ypred;
    end
end

posfit1 = posfit1(1:end-240,:);
posfit2 = posfit2(1:end-240,:);

time1 = time1(1:end-240,:);
time2 = time2(1:end-240,:);

%% check
figure(1)
subplot(3,1,1);plot(pos1(:,2));hold on;grid on;plot(posfit1(:,2),'-o');
subplot(3,1,2);plot(pos1(:,3));hold on;grid on;plot(posfit1(:,3),'-o');
subplot(3,1,3);plot(pos1(:,4));hold on;grid on;plot(posfit1(:,4),'-o');

figure(2)
subplot(3,1,1);plot(pos2(:,2));hold on;grid on;plot(posfit2(:,2),'-o');
subplot(3,1,2);plot(pos2(:,3));hold on;grid on;plot(posfit2(:,3),'-o');
subplot(3,1,3);plot(pos2(:,4));hold on;grid on;plot(posfit2(:,4),'-o');

pos1=posfit1;
pos2=posfit2;

save(fullfile(foldername,'position_rec.mat'),'pos1','pos2','time1','time2');



function data = txtread(txtfile)
    fid = fopen(txtfile,'r');
    l = fgetl(fid);%% skip first line
    num = 1;
    data = NaN(50000,9);
    while ~feof(fid)
        l = fgetl(fid);
        cntdata = split(l,',');
        datarow = str2num(str2mat(cntdata))';
        data(num,1:length(datarow)) = datarow;
        num = num + 1;
    end
    data(num:end,:) = [];
    fclose(fid);
end


