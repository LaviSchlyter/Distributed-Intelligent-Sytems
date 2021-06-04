close all
clear variables

supervisor_log_file = "../controllers/localization_supervisor/supervisor_log.csv";
controller_log_file = "../controllers/localization_controller/log_file.csv";

%% 1. Load true and approximated robot position
n_rob = 1;


[N_SIM_sup, T_SIM_sup, T_sup, pos_true] = read_log_project(supervisor_log_file);
[N_SIM, T_SIM, T, pos] = read_log_project(controller_log_file);

%This file contains: 
%       time, _pose.x, _pose.y , _KopiKopose.heading, _meas.gps[0], _meas.gps[1], 
%       _meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
%       _odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading, 
%       _kal_wheel.x, _kal_wheel.y, _kal_wheel.heading, _kal_acc.x, _kal_acc.y, _kal_acc.heading
%at each timestep.
%kal_weel: 19,20,21; kal_acc: 22,23,24

%% 3. Count number of robots, find timestep and #timesteps recorded
%find number of robots from supervisor csv
%n_rob = floor((size(true_pos_raw, 2)-1)/3);
ts = pos(1).time(2) - pos(1).time(1);
n_ts = floor(min(pos(1).time(end)/ts, pos_true(1).time(end)/ts));

%% 4. Extract information for each robot

% %true_pos: contains true x,y,heading (dim 2) at each timestep (dim 1) for  
% %each robot (dim 3)
% true_pos = zeros(n_ts, 3, n_rob);
% for i = 1:n_rob
%     true_pos(:,:,i) = true_pos_raw(1:n_ts,(3*i-1):(3*i+1));
% end

% %appr_pos: contains robot positions, but the origin is the robot initial
% %position. Change it to world origin (0,0,0)
% %gps y axis is inverted: correct. 
% pos(1).gps_y(:) = -pos(1).gps_y(:);
% pos(1).gps_x(:) = pos(1).gps_x(:) - pos.gps_x(1);
% pos(1).gps_y(:) = pos(1).gps_y(:) - pos.gps_y(1);

pos(1).true_x(:) = pos_true(1).true_x_rob0(1:n_ts) -...
    pos_true(1).true_x_rob0(1);
pos(1).true_y(:) = -(pos_true(1).true_y_rob0(1:n_ts) -...
    pos_true(1).true_y_rob0(1));
pos(1).true_heading(:) = pos_true(1).true_heading_rob0(1:n_ts) -...
    pos_true(1).true_heading_rob0(1);

% %verify that all lines in appr_pos have the same number of elements; if
% %not, truncate
% appr_pos_names = fieldnames(pos);
% for j = 1:n_rob
%     for i = 1:length(appr_pos_names)
%         l(i) = size(pos.(appr_pos_names{i}),1);
%         if length(unique(l)) > 1
%             %the column has different number of elements
%             pos.(appr_pos_names{i})(l(1):end) = [];
%         end
%         
%     end
% end
%% 4. plot trajectory

% plot true and estimated position for robot ind_rob
ind_rob = 1;
figure()
% add true trajectory (gps)
plot(pos(ind_rob).true_x(1:n_ts), pos(ind_rob).true_y(1:n_ts),'k')
% add trajectory calculated with kalman filter
hold on
plot(pos(ind_rob).kal_wheel_x(1:n_ts), pos(ind_rob).kal_wheel_y(1:n_ts),'b')
hold on
plot(pos(ind_rob).kal_acc_x(1:n_ts), pos(ind_rob).kal_acc_y(1:n_ts),'r')
% add trajectory calculated with odometry
plot(pos(ind_rob).odo_acc_x(1:n_ts), pos(ind_rob).odo_acc_y(1:n_ts),'y')
plot(pos(ind_rob).odo_enc_x(1:n_ts), pos(ind_rob).odo_enc_y(1:n_ts),'g')
% add title and legend
title("True and estimated trajectory")
legend({'True (GPS)','kalman wheel encoder','kalman acceleration',...
    'odo accelerometer','odo whell encoder'})

% plot estimated x,y,heading vs time for robot ind_rob
ind_rob = 1;
figure()
subplot(3,1,1)  %x
% add trajectory calculated with kalman filter
hold on
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_wheel_x(1:n_ts),'b')
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_acc_x(1:n_ts),'r')
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).gps_x(1:n_ts),'k')
% add title and legend
title("Estimated x")
legend({'est. kalman wheel encoder','est. kalman acceleration','true'})
subplot(3,1,2)  %y
% add trajectory calculated with kalman filter
hold on
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_wheel_y(1:n_ts),'b')
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_acc_y(1:n_ts),'r')
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).gps_y(1:n_ts),'k')
% add title and legend
title("Estimated y")
legend({'est. kalman wheel encoder','est. kalman acceleration','true'})
subplot(3,1,3)  %heading
% add trajectory calculated with kalman filter
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_wheel_heading(1:n_ts),'b')
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_acc_heading(1:n_ts),'r')
% add title and legend
title("Estimated heading")
legend({'est. kalman wheel encoder','est. kalman acceleration'})

% plot estimated heading vs time for robot ind_rob
ind_rob = 1;
figure()
% add trajectory calculated with kalman filter
hold on
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_wheel_heading(1:n_ts),'b')
hold on
plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_acc_heading(1:n_ts),'r')
% add title and legend
title("Estimated heading")
legend({'est. kalman wheel encoder','est. kalman acceleration'})

%% 5. Compute metrics
% Necessary:
vmax = 6.28; %robot max speed in webots
g = [0,0;1,1]; %target robot positions in local frame attached to leader = robot 1
%???????????????

%compute the metrics
[fit_loc_wheel] = fit_loc(pos,...
    {'kal_wheel_x','kal_wheel_y','kal_wheel_heading'},n_ts, n_rob, vmax, g, ts);
[fit_loc_acc] = fit_loc(pos,...
     {'kal_acc_x','kal_acc_y','kal_acc_heading'},n_ts, n_rob, vmax, g, ts);

%plot the metrics
% plot localization performance metric over time
figure()
plot(pos.time(1:n_ts), fit_loc_wheel,'b')
hold on
plot(pos.time(1:n_ts), fit_loc_acc,'r')
title("Localization fitness")
legend({'est. kalman wheel encoder','est. kalman acceleration'})

%get average value
avg_fit_loc_wheel = mean(fit_loc_wheel);
avg_fit_loc_wheel = mean(fit_loc_wheel);

