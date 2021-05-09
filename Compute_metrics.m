close all
%Add all subfolders to working directory
folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

%% 1. Load true robot position 
true_pos_raw = csvread("Robots_true_position.csv",0,0); 

%Do not read the last column as it contains useless data
%This file contains: time,x,y,z of each robot at each timestep

%% 2. Load approximated robot positions
[fid,msg] = fopen('log_file.csv','rt');
assert(fid>=3,msg)
appr_pos_raw = textscan(fid,"%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f",...
    'Delimiter',';','HeaderLines',1);
fclose(fid);
%This file contains: 
      %time, _pose.x, _pose.y , _pose.heading, _meas.gps[0], _meas.gps[1], 
      %_meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
      %_odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading, 
      %_kal_wheel.x, _kal_wheel.y, _kal_wheel.heading, _kal_acc.x, _kal_acc.y, _kal_acc.heading
%at each timestep.
%kal_weel: 19,20,21; kal_acc: 22,23,24

%% 3. Count number of robots, find timestep and #timesteps recorded
%find number of robots from supervisor csv
n_rob = floor((size(true_pos_raw, 2)-1)/3);

%time from supervizor is in milisecond: convert to second
time_true = true_pos_raw(:,1)/1000; %[s]
time_appr = appr_pos_raw{:,1};      %[s]
%find time of last timestep recorded
tf = min(max(time_true), max(time_appr));
%find timestep length
ts = appr_pos_raw{1}(2) - appr_pos_raw{1}(1);
%fond total number of timesteps
n_ts = tf/ts+1;

%% 4. Extract information for each robot
%time vector: 
time = 0:ts:tf;
time = time';

%true_pos: contains true x,y,heading (dim 2) at each timestep (dim 1) for  
%each robot (dim 3)
true_pos = zeros(n_ts, 3, n_rob);
for i = 1:n_rob
    true_pos(:,:,i) = true_pos_raw(1:n_ts,(3*i-1):(3*i+1));
end

%true_pos: contains kalman x,y,heading (dim 2) at each timestep (dim 1) for  
%each robot (dim 3)
kalman_w_pos = zeros(n_ts, 3, n_rob);
kalman_a_pos = zeros(n_ts, 3, n_rob);
gps_pos = zeros(n_ts, 3, n_rob);
for i = 1:n_rob
    kalman_w_pos(:,:,i) = [appr_pos_raw{19}(1:n_ts),...
        appr_pos_raw{20}(1:n_ts), appr_pos_raw{21}(1:n_ts)];
    kalman_a_pos(:,:,i) = [appr_pos_raw{22}(1:n_ts),...
        appr_pos_raw{23}(1:n_ts), appr_pos_raw{24}(1:n_ts)];  
    gps_pos(:,:,i) = [appr_pos_raw{5}(1:n_ts),...
        appr_pos_raw{6}(1:n_ts), appr_pos_raw{6}(1:n_ts)];
end


%% 4. plot trajectory

% plot true and estimated position for robot ind_rob
ind_rob = 1;
figure()
% add true trajectory 
plot(true_pos(:,1,ind_rob), true_pos(:,2,ind_rob),'k')
% add trajectory calculated with GPS
hold on
plot(gps_pos(:,1,ind_rob), gps_pos(:,2,ind_rob),'b')
% add trajectory calculated with kalman filter
hold on
plot(kalman_w_pos(:,1,ind_rob), kalman_w_pos(:,2,ind_rob),'y')
hold on
plot(kalman_a_pos(:,1,ind_rob), kalman_a_pos(:,2,ind_rob),'r')
% add title and legend
title("True and estimated trajectory")
legend({'True','GPS','kalman wheel encoder','kalman acceleration'})

% plot true and estimated heading vs time for robot ind_rob
ind_rob = 1;
figure()
% add true trajectory 
plot(time, true_pos(:,3,ind_rob),'k')
% add trajectory calculated with kalman filter
hold on
plot(time, kalman_w_pos(:,3,ind_rob),'b')
hold on
plot(time, kalman_a_pos(:,3,ind_rob),'r')
% add title and legend
title("True and estimated heading")
legend({'True','est. kalman wheel encoder','est. kalman acceleration'})

%% 5. Compute metrics
% Necessary:
Dfl = 0.5; %target flocking distance [m]
vmax = 0.6; %robot max speed in webots
g = [0,0;1,1]; %target robot positions in local frame attached to leader = robot 1
%???????????????

%performance metric: localization
fit_loc = zeros(n_ts,1);
for i = 1:n_ts
    for j = 1:n_rob
    fit_loc(i) = sqrt((true_pos(i,1,j)-kalman_a_pos(i,1,j))^2+...
        (true_pos(i,2,j)-kalman_a_pos(i,2,j))^2);
    end
end

%performance metric: flocking
%calculate o(t): orientation between robots
%calculate dfl(t): distance between robots
o = zeros(n_ts,1);
dfl = zeros(n_ts,1);
dfl1 = zeros(n_ts,1);
dfl2 = zeros(n_ts,1);
flock_center_pos = zeros(n_ts,2);
v = zeros(n_ts,1);
for i = 1:n_ts
    flock_center_pos(i,1) = mean(true_pos(i,1,:));
    flock_center_pos(i,2) = mean(true_pos(i,2,:));

    for j = 1:n_rob
        if i == 1
            v(i) = 0;
        else
            v(i) = sqrt((true_pos(i-1,1,j)-true_pos(i-1,1,j))^2+...
                (true_pos(i,2,j)-true_pos(i-1,2,j))^2)/(vmax*ts);
        end
        dfl1(i) = dfl1(i) + sqrt((true_pos(i,1,j)-flock_center_pos(i,1))^2+...
        (true_pos(i,2,j)-flock_center_pos(i,2))^2);
        for k = (j+1):n_rob
            o(i) = o(i) + abs(true_pos(i,3,j)-kalman_a_pos(i,3,j))/pi;
            %inter-robot distance of pair j,k
            delta_xj = sqrt((true_pos(i,1,j)-true_pos(i,1,k))^2+...
                (true_pos(i,2,j)-true_pos(i,1,k))^2); 
            %target inter-robot distance of pair j,k
            Dfl = sqrt((g(j,1)-g(k,1))^2 + (g(j,2)-g(k,2))^2);
            dfl2(i) = dfl2(i) + min(delta_xj/Dfl, 1/(1-Dfl+delta_xj)^2);
        end
    end
    o(i) = 1 - o(i)/(n_rob*(n_rob-1)/2);
    dfl1(i) = 1/(1 + dfl1(i)/n_rob);
    dfl2(i) = dfl2(i)/n_rob;
    dfl(i) = dfl1(i) + dfl2(i);
end
fit_flocking = o.*dfl.*v;

%performance metric: formation
%for this step, we need robot coordinates in local reference frame; origin
%= leader = first robot.
loc_true_pos = zeros(size(true_pos));
for i = 2:n_rob %leader (i = 1) keeps pos = 0
     loc_true_pos(:,:,i) = true_pos(:,:,i) - true_pos(:,:,1);
end
dfo = zeros(n_ts,1);
for i = 1:n_ts
    for j = 1:n_rob
        dfo(i) = dfo(i) + sqrt((loc_true_pos(i,1,j)-g(j,1))^2+...
                (loc_true_pos(i,2,j)-g(j,2))^2); 
    end
    dfo(i) = 1 + dfo(i)/n_rob;
end
fit_formation = dfo.*v;