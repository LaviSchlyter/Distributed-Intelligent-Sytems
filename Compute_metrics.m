close all
%Add all subfolders to working directory
folder = fileparts(which(mfilename)); 
addpath(genpath(folder));

%% 1. Load true robot position 
true_pos = csvread("Robots_true_position.csv",0,0); 


%Do not read the last column as it contains useless data
%This file contains: x,y,z of each robot at each timestep

%count number of robots
%% 2. Load approximated robot positions
[fid,msg] = fopen('log_file.csv','rt');
assert(fid>=3,msg)
appr_pos = textscan(fid,"%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f%f",...
    'Delimiter',';','HeaderLines',1);
fclose(fid);
%This file contains: 
      %time, _pose.x, _pose.y , _pose.heading, _meas.gps[0], _meas.gps[1], 
      %_meas.gps[2], _meas.acc[0], _meas.acc[1], _meas.acc[2], _meas.right_enc, _meas.left_enc, 
      %_odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading, 
      %_kal_wheel.x, _kal_wheel.y, _kal_wheel.heading, _kal_acc.x, _kal_acc.y, _kal_acc.heading
%at each timestep.
  
%% 3. Count number of robots, find timestep and #timesteps recorded
n_rob = floor((size(true_pos, 2)-1)/3);
ts = appr_pos{1}(2) - appr_pos{1}(1);
n_ts = size(true_pos,1);

%% 4. plot trajectory

% plot true trajectory
figure()
plot(true_pos(1:n_ts,2), true_pos(1:n_ts,3),'b')
title("True and estimated trajectory")
% add trajectory calculated with kalman filter
hold on
plot(appr_pos{22}(1:n_ts), appr_pos{23}(1:n_ts), 'r')
% add trajectory calculated with odometry (acceleration) alone
hold on
plot(appr_pos{14}(1:n_ts), appr_pos{15}(1:n_ts), 'y')
% add trajectory calculated with odometry (wheel encoders) alone
hold on
plot(appr_pos{17}(1:n_ts), appr_pos{18}(1:n_ts), 'g')
% add trajectory calculated with gps alone
hold on
plot(appr_pos{5}(1:n_ts), appr_pos{6}(1:n_ts), 'k')
legend({'True','est. kalman','est. odo. acc','est. odo. wheelenc','est. gps'})

% plot true heading
figure()
plot(true_pos(1:n_ts,4),'b')
title("Heading over time")
% add heading calculated with kalman filter
hold on
plot(appr_pos{24}(1:n_ts), 'r')
% add trajectory calculated with odometry (acceleration) alone
hold on
plot(appr_pos{16}(1:n_ts), 'y')
% add trajectory calculated with odometry (wheel encoders) alone
hold on
plot(appr_pos{19}(1:n_ts), 'g')
% add trajectory calculated with gps alone
hold on
plot(appr_pos{7}(1:n_ts), 'k')
legend({'True','est. kalman','est. odo. acc','est. odo. wheelenc','est. gps'})


%4. Plot metrics




% %%
% /*
%  * Compute performance metric. called at each timestep.
%  */
% void compute_fitness(float* fit_loc, float* fit_flocking, float* fit_formation) {
% 	// Compute performance indices for all robots at once.
% 
% 	//performance metric: localization
% 	*fit_loc = 0;
% 	int i; int j;
% 	for (i=0;i<FLOCK_SIZE;i++) {
% 		//sqrt((x-xapr)^2 + (z-zapr)^2)
% 		*fit_loc = sqrt(pow(loc[i][0]-apr_loc[i][0],2)+pow(loc[i][1]-apr_loc[i][1],2));
% 	}
% 	
% 	//performance metric: flocking
% 	*fit_flocking = 0;
% 	float o = 0;
% 	float dfl = 0;
% 	float v = 0;
% 	//calculate o(t): difference of heading between all pairs
%            for (i=0;i<FLOCK_SIZE;i++) {
% 		for (j=i+1;j<FLOCK_SIZE;j++) {	
% 			// Distance measure for each pair of robots
% 			o += abs(loc[i][2]-loc[j][2]);
% 		}
% 		o /= 1;//FLOCK_SIZE*(FLOCK_SIZE-1)/2;uncomment whenever we have more than 1 robot in the flock!
% 		o = 1-o;
% 	}
% 	//calculate dfl(t): distance between robots
%            float ctr_x = 0;
%            float ctr_z = 0;
%            float prev_ctr_x = 0;
%            float prev_ctr_z = 0;
%            float delta_xj = 0;
%            float dfl1 = 0; //represent the two members (parenthesis) of the metric
%            float dfl2 = 0; //calculate position of the flock center
%            for (i=0;i<FLOCK_SIZE;i++) {
%                      ctr_x += loc[i][0];
%                      ctr_z += loc[i][1];
%            }
%            ctr_x /= FLOCK_SIZE;
%            ctr_z /= FLOCK_SIZE;
%            //calculate previous position of the flock center
%            for (i=0;i<FLOCK_SIZE;i++) {
%                      prev_ctr_x += prev_loc[i][0];
%                      prev_ctr_z += prev_loc[i][1];
%            }
%            prev_ctr_x /= FLOCK_SIZE;
%            prev_ctr_z /= FLOCK_SIZE;
%            //first parenthesis
%            for (i=0;i<FLOCK_SIZE;i++) {
%                      dfl1 += sqrt(pow(loc[i][0]-ctr_x,2)+pow(loc[i][1]-ctr_z,2));
%            }
%            dfl1 /= FLOCK_SIZE;
%            dfl1 = 1/dfl1;
%            //secod parenthesis
%            for (i=0;i<FLOCK_SIZE;i++) {
% 		for (j=i+1;j<FLOCK_SIZE;j++) {
% 		delta_xj = sqrt(pow(loc[i][0]-loc[j][0],2)+pow(loc[i][1]-loc[j][1],2)); //inter-robot distance of the pair	
% 		dfl2 += fmin(delta_xj/TARGET_FLOCKING_DISTANCE, 1/pow(1-TARGET_FLOCKING_DISTANCE+delta_xj, 2));
% 		}  
% 	} 
% 	dfl2 /= FLOCK_SIZE;
% 	//compute dfl
% 	dfl = dfl1*dfl2;     
% 	//calculate v(t)
% 	v = sqrt(pow(ctr_x - prev_ctr_x,2)+pow(ctr_z - prev_ctr_z,2));
% 	v /= MAX_SPEED_WEB*TIME_STEP;
% 	//compute final flocking metric
% 	*fit_flocking = o*dfl*v;
% 	
% 	//performance metric: formation
% 	*fit_formation = 0;
% 	float dfo = 0;
% 	//relative distances are measured with leader as the origin (leader = e-puck0)
% 	for (i=0;i<FLOCK_SIZE;i++) {
%     	           rel_loc[i][0] = loc[i][0] - loc[0][0];
%     	           rel_loc[i][1] = loc[i][1] - loc[0][1];
% 		dfo = sqrt(pow(rel_loc[i][0]-target_rel_loc[i][0],2)+pow(rel_loc[i][1]-target_rel_loc[i][1],2)); //inter-robot distance of the pair	
% 	}
% 	//compute final formation metric
% 	*fit_flocking = dfo*v;
% 	
% 	printf("o %f, dfl1 %f, dfl2 %f, v %f, dfo %f\n", o, dfl1, dfl2, v, dfo);	       
% 	printf("loc[0] %f, loc[1] %f, loc[2] %f", loc[0], loc[1], loc[2]);
% 	printf("apr_loc[0] %f, apr_loc[1] %f, apr_loc[2] %f", apr_loc[0], apr_loc[1], apr_loc[2]);
% 	printf("loc[0] %f, loc[1] %f, loc[2] %f", loc[0], loc[1], loc[2]);
% }