close all
clear variables

supervisor_log_file = "../controllers/localization_supervisor/supervisor_log.csv";
controller_log_file = "../controllers/localization_controller/log_file.csv";

%% 1. Load true and approximated robot position
n_rob1 = 1; %expected number of robots
[N_SIM_sup, T_SIM_sup, T_sup, pos_true] = read_log_project(supervisor_log_file);
[N_SIM, T_SIM, T, pos] = read_log_project(controller_log_file);

%% 3. Count number of robots, find timestep and #timesteps recorded
%find number of robots from supervisor csv
n_rob = (numel(fieldnames(pos_true(1)))-1)/3;
if n_rob1 ~= n_rob
    sprintf("Error! Expected number of robots does not match the number of robots in the log file.\n")
    return;
end
ts = pos(1).time(2) - pos(1).time(1);
n_ts = floor(min(pos(1).time(end)/ts, pos_true(1).time(end)/ts));

%% 4. Extract information for each robot
% store true information in pos structure; invert y axis and store in local
% coordinates (like Kalman output).
pos(1).true_x(:) = pos_true(1).true_x_rob0(1:n_ts) -...
    pos_true(1).true_x_rob0(1);
pos(1).true_y(:) = -(pos_true(1).true_y_rob0(1:n_ts) -...
    pos_true(1).true_y_rob0(1));
pos(1).true_heading(:) = pos_true(1).true_heading_rob0(1:n_ts) -...
    pos_true(1).true_heading_rob0(1);

% store GPS (accessible by robots, refreshed each second) in local frame.
pos(1).gps_x_shifted(:) = pos(1).gps_x(1:n_ts) -...
    pos_true(1).true_x_rob0(1);
pos(1).gps_y_shifted(:) = -(pos(1).gps_y(1:n_ts) -...
    pos_true(1).true_y_rob0(1));
pos(1).gps_x_shifted(pos(1).gps_x_shifted == -pos_true(1).true_x_rob0(1))= 0;
pos(1).gps_y_shifted(pos(1).gps_y_shifted == -pos_true(1).true_y_rob0(1))= 0;

%% 4. plot trajectory
% plot GPS each second --> find timesteps corresponding to round time
% measurements
[idx_gps,~,~] = find(abs(pos(1).time(:)-floor(pos(1).time(:)))<ts*0.9);
idx_gps(idx_gps > n_ts) = [];

% plot true and estimated position for robot ind_rob
ind_rob = 1;
f = figure();
% add true trajectory (gps)
c_red = [203,24,29];
c_red = c_red/256;
c_cyan = [33,113,181];
c_cyan = c_cyan/256;
plot(pos(ind_rob).true_x(idx_gps), pos(ind_rob).true_y(idx_gps),'kx',...
    'MarkerSize',5)
% add trajectory calculated with kalman filter
hold on
plot(pos(ind_rob).kal_wheel_x(1:n_ts), pos(ind_rob).kal_wheel_y(1:n_ts),...
    'Color',c_cyan)
plot(pos(ind_rob).odo_enc_x(1:n_ts), pos(ind_rob).odo_enc_y(1:n_ts),'LineStyle','--',...
    'Color',c_cyan)
plot(pos(ind_rob).kal_acc_x(1:n_ts), pos(ind_rob).kal_acc_y(1:n_ts),'Color',c_red)
% add trajectory calculated with odometry
plot(pos(ind_rob).odo_acc_x(1:n_ts), pos(ind_rob).odo_acc_y(1:n_ts),'Color',c_red,...
    'LineStyle','--')
plot(pos(ind_rob).gps_x_shifted(1:n_ts), pos(ind_rob).gps_y_shifted(1:n_ts),'k',...
    'LineStyle','--')
% add title and legend
legend({'True trajectory','Kalman wheel encoder','Odometry wheel encoder',...
    'Kalman acceleration',...
    'Odometry accelerometer'})
xlim([0,1.4])
xlabel('X coordinate [m]')
ylabel('Y coordinate [m]')
saveas(f,'Figures/LocPosition.png')

%% 5. Compute metrics
%compute the metrics
[fit_loc_wheel] = fit_loc(pos,...
    {'odo_enc_x','odo_enc_y','odo_enc_heading'},n_ts, n_rob);
[fit_loc_acc] = fit_loc(pos,...
     {'odo_acc_x','odo_acc_y','odo_acc_heading'},n_ts, n_rob);
[fit_loc_wheel_kal] = fit_loc(pos,...
    {'kal_wheel_x','kal_wheel_y','kal_wheel_heading'},n_ts, n_rob);
[fit_loc_acc_kal] = fit_loc(pos,...
     {'kal_acc_x','kal_acc_y','kal_acc_heading'},n_ts, n_rob);
[fit_loc_GPS_shifted] = fit_loc(pos,...
     {'gps_x_shifted','gps_y_shifted'},n_ts, n_rob);
 
%plot the metrics
% plot localization performance metric over time
f = figure();
hax = axes;
hold on
plot(pos.time(1:n_ts), fit_loc_wheel,'Color',c_cyan,'LineStyle','--','LineWidth',1.2)
hold on
plot(pos.time(1:n_ts), fit_loc_acc,'Color',c_red, 'LineStyle','--','LineWidth',1.2)
plot(pos.time(1:n_ts), fit_loc_wheel_kal,'Color',c_cyan,'LineWidth',1.2)
hold on
plot(pos.time(1:n_ts), fit_loc_acc_kal,'Color',c_red,'LineWidth',1.2)
hold on
plot(pos.time(1:n_ts), fit_loc_GPS_shifted,"xk" )
xlabel("Run time [s]")
ylabel("Localization fitness metric [-]")
hold on
% Color turns in red
box1=[1 1 3 3];
box2=[14 14 16 16];
boxy=[0 1 1 0]*max(fit_loc_acc)*1.2;
patch(box1,boxy,[1 0 0],'FaceAlpha',0.2,'LineStyle','None')
patch(box2,boxy,[1 0 0],'FaceAlpha',0.2,'LineStyle','None')
ylim([0 max(fit_loc_acc)*1.2])
xlim([0, n_ts*0.016])
% Legend
legend({'Odometry: wheel encoder','Odometry: accelerometer', ...
    'Kalman wheel encoder','Kalman acceleration', 'GPS shifted'})
saveas(f,'Figures/LocFitness.png')

%get average value
avg_fit_loc_acc = mean(fit_loc_acc);
avg_fit_loc_wheel = mean(fit_loc_wheel);

