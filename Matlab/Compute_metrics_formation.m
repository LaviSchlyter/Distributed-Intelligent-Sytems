close all
clear variables

path_leader = "../controllers/crossing_leader";
path_follower = "../controllers/crossing_follower_mataric";
path_supervisor = "../controllers/localization_supervisor/supervisor_log.csv";

%% 1. Load supervisor log file

[N_SIM, T_SIM, T, pos_true] = read_log_project(path_supervisor);
rob_idx = [0,1,2,3,4,5,6,7,8,9];
leader_idx = [0,5];
% %% 2.a Load leader log files
% 
% % user knowledge: 
% leader_idx = [0,5];
% formation = [1 2 1 1 2 2];
% 
% % count number of leaders
% folder_info = dir(path_leader);
% n_leader = 0;
% leader_log_file_names = {};
% for i = 1:length(folder_info)
%     if contains(folder_info(i).name, 'log_file')
%         n_leader = n_leader + 1;
%         leader_log_file_names{i} = folder_info(i).name;
%     end
% end
% %remove empty entries
% leader_log_file_names = leader_log_file_names(~cellfun('isempty',...
%     leader_log_file_names));
% 
% %filename = strcat(path_leader,"/log_file_robot0.csv");
% for i = 1:n_leader
%     filename = strcat(path_leader,'/');
%     filename = strcat(filename,leader_log_file_names{i});
%     [N_SIM, T_SIM, T, pos(i)] = read_log_project(filename);
% end
% 
% % %% 2. Load follower log files
% % 
% % % count number of followers
% % follower_folder_info = dir(path_follower);
% % n_follower = 0;
% % follower_log_file_names = {};
% % for i = 1:length(follower_folder_info)
%     if contains(follower_folder_info(i).name, 'log_file')
%         n_leader = n_leader + 1;
%         follower_log_file_names{i} = follower_folder_info(i).name;
%     end
% end
% %remove empty entries
% follower_log_file_names = follower_log_file_names(~cellfun('isempty',...
%     follower_log_file_names));
% % retrieve information for each log file
% for i = 1:n_follower
%     filename = strcat(path_follower,'/');
%     filename = strcat(filename,follower_log_file_names{i});
%     [N_SIM, T_SIM, T, pos(n_leader+i)] = read_log_project(filename);
% end


%% 3. Count number of robots, find timestep and #timesteps recorded
%find number of robots from supervisor csv
n_rob = (numel(fieldnames(pos_true(1)))-1)/3;

for i = 1:n_rob
    pos(i).time = pos_true(1).time;
    pos(i).x = pos_true(1).(sprintf("true_x_rob%d",rob_idx(i)))(1:end);
    pos(i).y = -pos_true(1).(sprintf("true_y_rob%d",rob_idx(i)))(1:end);
    pos(i).heading = pos_true(1).(sprintf("true_heading_rob%d",rob_idx(i)))(1:end);
end

ts = pos(1).time(2) - pos(1).time(1);
n_ts = floor(pos(1).time(end)/ts);

% 
% for i = 1:n_rob
%     pos(i).true_x(:) = pos(i).(sprintf("true_x_rob%d",leader_idx(i)))(1:n_ts);
%     pos(i).true_y(:) = -(pos(i).(sprintf("true_y_rob%d",leader_idx(i)))(1:n_ts));
%     pos(i).true_heading(:) = pos(i).(sprintf("true_heading_rob%d",leader_idx(i)))(1:n_ts);
% end


%% 4. plot trajectory

% plot true and estimated position for each robot
figure()
hold on
for i = 1:n_rob
    % add true trajectory (gps)
    plot(pos(i).x(1:n_ts), pos(i).y(1:n_ts),'k')
    % add title and legend
    title("Robot trajectory")
    legend({'True (GPS)','kalman wheel encoder'})
end
% 
% % plot estimated x,y,heading vs time for robot ind_rob
% ind_rob = 1;
% figure()
% subplot(3,1,1)  %x
% % add trajectory calculated with kalman filter
% hold on
% plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_wheel_x(1:n_ts),'b')
% plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).gps_x(1:n_ts),'k')
% % add title and legend
% title("Estimated x")
% legend({'est. kalman wheel encoder','true'})
% subplot(3,1,2)  %y
% % add trajectory calculated with kalman filter
% hold on
% plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_wheel_y(1:n_ts),'b')
% plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).gps_y(1:n_ts),'k')
% % add title and legend
% title("Estimated y")
% legend({'est. kalman wheel encoder','true'})
% subplot(3,1,3)  %heading
% % add trajectory calculated with kalman filter
% hold on
% plot(pos(ind_rob).time(1:n_ts), pos(ind_rob).kal_wheel_heading(1:n_ts),'b')
% % add title and legend
% title("Estimated heading")
% legend({'est. kalman wheel encoder','est. kalman acceleration'})
% 
%% 5. Compute metrics
target_range = 0.1414;
target_bearing = [pi/4 , 7*pi/4 , pi/4 , 7*pi/4];
g = [0,0;...
    cos(target_bearing(1))*target_range, sin(target_bearing(1))*target_range;...
    cos(target_bearing(2))*target_range, sin(target_bearing(2))*target_range;...
    2*cos(target_bearing(3))*target_range, 2*sin(target_bearing(3))*target_range;...
    2*cos(target_bearing(4))*target_range, 2*sin(target_bearing(4))*target_range;... 
    0,0;...
    -cos(target_bearing(1))*target_range, sin(target_bearing(1))*target_range;...
    -cos(target_bearing(2))*target_range, sin(target_bearing(2))*target_range;...
    -2*cos(target_bearing(3))*target_range, 2*sin(target_bearing(3))*target_range;...
    -2*cos(target_bearing(4))*target_range, 2*sin(target_bearing(4))*target_range;...      
   ]; %target robot positions in local frame attached to each leader
% g = [0,0;0.1,0.1;0.1,-0.1;0.2,0.2;0.2,-0.2]

leader_ofeachrob = [1 1 1 1 1; 5 5 5 5 5];
team = [1,1,1,1,1,2,2,2,2,2];

figure()
scatter(g(1:5,1), g(1:5,2),'r');
hold on
scatter(g(6:end,1), g(6:end,2),'b');
legend({'leader = 0','leader = 5'});

% Necessary:
vmax = 6.28/100; %robot max speed in webots

%compute the metrics
n_team = 2;
[fit_form, dfo, v, flock_ctr_x, flock_ctr_y] = fit_formation(pos,n_ts,n_rob,n_team,vmax,g,...
    leader_ofeachrob,ts,team);

%plot the metrics
% plot localization performance metric over time
figure()
plot(pos(1).time(1:n_ts), fit_form)
hold on
plot(pos(1).time(1:n_ts), v)
plot(pos(1).time(1:n_ts), dfo)
legend({'formation fitness','v','dfo'})

figure()
hold on
plot(flock_ctr_x(1,:),flock_ctr_y(1,:),'r')
plot(flock_ctr_x(2,:),flock_ctr_y(2,:),'b')
title("Foramtion center as f(t)")
legend({'formation 1 (epuck 0 to 4)','formation 2 (epuck 5 to 9)'})
% 
% %get average value
% avg_fit_loc_wheel = mean(fit_loc_wheel)
% 
% 
% % what kind of visual? robot x/y position at each time for both teams with different colors +
% % flocking/formation/localization metrics as f(t). Confirm with Tiff.
% 
% % supervisor: write logs for all followers. 
% 
% % formation (world "crossing")
% % pour chaque type de controlleurs:
% % - mataric VS laplacian
% % - P VS PI (VS PINL?) (bool)
% % - number of edges (for the laplacian) (explained in the code)
% % - number of robots (3,4,5)
% % plot fitness_formation(t) and compare performance.
% 
% % flocking (world "flocking")
% % pour chaque type de controlleurs:
% % - flocking with optimized weights VS flocking with manual weights
% % - number of robots (3,4,5)
% % plot fitness_flocking(t) and compare performance.
