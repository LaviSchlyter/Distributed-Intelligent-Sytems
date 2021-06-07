close all
clear variables

% controller type and flocking parameters:
c_type = 'laplacian_10e_base'; 
%controller type (mataric or laplacian) must 
% be included. This will be used to generate the name of stored metrics
% values; make sure to use different names. The best s to precise
% mataric/laplacian, P/PI/NPI or number of edges, and PSO or not.
n_rob1 = 5;  %expected number of robots
rob_idx = [0,1,2,3,4];  %robot names (epuck + rob_idx)
leader_idx = [0];   %leader name (epuck + rob_idx)

path_leader = "../controllers/obstacle_leader";
if contains(c_type,'laplacian')
    path_follower = "../controllers/obstacle_follower_laplacian";
else
    if contains(c_type,'mataric')
        path_follower = "../controllers/obstacle_follower_mataric";
    else
        sprintf('Unknown controller type.\n');
    end
end
path_supervisor = "../controllers/localization_supervisor/supervisor_log.csv";

%% 1. Load supervisor log file
% to obtain true positions.
[N_SIM, T_SIM, T, pos_true] = read_log_project(path_supervisor);

% Number of robots
n_rob = (numel(fieldnames(pos_true(1)))-1)/3;

if n_rob ~= n_rob1
    sprintf("error! the number of recorded robots is not what we thought...\n")
    return
end
%% 2. Count number of robots, find timestep and #timesteps recorded
%find number of robots from supervisor csv

%integrate supervisor coordinates in pos data
for i = 1:n_rob
    pos(i).time = pos_true(1).time;
    pos(i).x = pos_true(1).(sprintf("true_x_rob%d",rob_idx(i)))(1:end);
    pos(i).y = -pos_true(1).(sprintf("true_y_rob%d",rob_idx(i)))(1:end);
    pos(i).heading = pos_true(1).(sprintf("true_heading_rob%d",rob_idx(i)))(1:end);
end

rob_idx = rob_idx + 1; %correct for indexing differences between c and matlab
ts = pos(1).time(2) - pos(1).time(1);
n_ts = floor(pos(1).time(end)/ts);


%% 2. Compute metrics

target_range = 0.082*ones(n_rob,n_rob);

% Necessary:
vmax_rad = 6.28; %robot max speed in webots = 6.28 cm/s; 
vmax = vmax_rad * 0.0205;

%compute the metrics
[fit_flock, o, dfl, v, dfl1, dfl2, flock_ctr_x, flock_ctr_y] = ...
    fit_flocking(pos,n_ts,n_rob, target_range, vmax, ts);
save(sprintf('Metric_values/flock_%s_%d', c_type, n_rob),'fit_flock')

%plot the metrics over time
f = figure();
title("Formation performance over time")
plot(pos(1).time(1:n_ts), fit_flock, 'k')
hold on
plot(pos(1).time(1:n_ts), o, 'b')
plot(pos(1).time(1:n_ts), dfl, 'r')
plot(pos(1).time(1:n_ts), v, 'g')
legend({'flocking fitness','o (common orientation of the robots)','dfl (respect of robots for target range)','v (speed of flock towards objective)'})
xlabel("Simulation time [s]")
ylabel("Performance metric [-]")

mean(fit_flock(1:n_ts));

