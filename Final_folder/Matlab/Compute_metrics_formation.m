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
n_teams = 1;    %number of different robots flocks
world = 'obstacle';

path_leader = strcat("../controllers/%s_leader",world);
if contains(c_type,'laplacian')
    path_follower = strcat("../controllers/%s__follower_laplacian",world);
else
    if contains(c_type, 'mataric')
        path_follower = strcat("../controllers/%s__follower_mataric",world);
    else
        sprintf('Unknown controller type.\n');
    end
end

path_supervisor = "../controllers/localization_supervisor/supervisor_log.csv";

%% 1. Load supervisor log file
[N_SIM, T_SIM, T, pos_true] = read_log_project(path_supervisor);

% Number of robots
n_rob = (numel(fieldnames(pos_true(1)))-1)/3;

if n_rob ~= n_rob1
    sprintf("error! the number of recorded robots is not what we thought...\n")
    return
end

col_names = fieldnames(pos_true(1));
rob_idx = [];
pos_rob = zeros(n_rob,1);
k = 1;
for i = 2:3:length(col_names)
    rob_idx = [rob_idx str2num(col_names{i}(end))];
    pos_rob(str2num(col_names{i}(end))+1) = k;
    k = k +1;
end
% Leader id's 
leader_idx = [0,5];

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

%% 3. Compute metrics
sprintf("Simulation: \nController type: %s\nNumber of teams: %d\nNumber of robots: %d",...
    c_type, n_teams, n_rob)

% Target values between robots
target_range = 0.1414;
target_bearing = [pi/4 , 7*pi/4 , pi/4 , 7*pi/4];


% For crossing world 
if contains(world,'crossing')
    leader_ofeachrob = [1 1 1 1 1, 6 6 6 6 6];
team = [1,1,1,1,1,2,2,2,2,2];
n_rob_perTeam = n_rob/n_teams;
    g = [0,0;...
        0.1,0.1;...
        0.1,-0.1;...
        0.2,0.2;...
        0.2,-0.2;...
        0,0;...
        -0.1,-0.1;...
        -0.1,0.1;...
        -0.2,-0.2;...
        -0.2,0.2];
end
% For obstacle world
if contains(world,'obstacle')
    leader_ofeachrob = [1 1 1 1 1 1 1];
    team = [1,1,1,1,1,1,1];
n_rob_perTeam = n_rob/n_teams;
    g = [0,0;...
     -0.1,-0.1;...
     -0.1,0.1;...
     -0.2,-0.2;...
     -0.2,0.2;...
     -0.3,-0.1;...
     -0.3,0.1];
 g(n_rob_perTeam+1:7,:) = [];
 leader_ofeachrob(n_rob_perTeam+1:7) = [];
 team(n_rob_perTeam+1:7) = [];
end

% Necessary:
vmax_rad = 6.28; %robot max speed in webots [rad/s]
wheel_radius = 0.02;
vmax = vmax_rad * wheel_radius;

% Compute the metrics
n_team = 2;
[fit_form, dfo, v, flock_ctr_x, flock_ctr_y, loc_pos_x, loc_pos_y] = ...
    fit_formation(pos,n_ts,n_rob,n_teams,vmax,g,...
    leader_ofeachrob,ts,team, rob_idx, pos_rob);

% Plot the metrics
% Plot formation performance metric over time
f = figure();
subplot(2,1,1)
plot(pos(1).time(1:n_ts), fit_form, 'k')
title(sprintf("Formation fitness over time (%s, %d rob)", c_type, n_rob))
xlabel("Time [s]")
ylabel("Metric value [-]")
legend({'formation fitness'})
subplot(2,1,2)
plot(pos(1).time(1:n_ts), v, 'b')
hold on
plot(pos(1).time(1:n_ts), dfo, 'r')
title("Metrics component")
xlabel("Time [s]")
ylabel("Metric value [-]")
legend({'v (speed towards objective)','dfo (respect of formation)'})

% Save the metrics value
save(sprintf('Metric_values/form_%s_%d', c_type, n_rob),'fit_form')

% Compute the mean fitness for formation 
mean(fit_form);
