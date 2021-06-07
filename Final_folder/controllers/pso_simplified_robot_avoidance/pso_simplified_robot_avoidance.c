/*****************************************************************************************************************************/
/* File:         pso_simplified_robot_flock.c                                   */
/* Version:      1.0                                                            */
/* Date:         01-Mai-21                                                      */
/* Description:  PSO for the avoidance with a public individual heterogeneous   */
/*               configuration. Robot controller based on robot_flocking.c      */
/*                                                                              */
/* Author:       Paco Mermoud                                                   */
/*****************************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/position_sensor.h>

// L.
#include "../localization_controller/utils.h"
#include "../localization_controller/odometry.h"
#include "../localization_controller/kalman.h"

// ------------------------- Choose the flock size  -------------------------
#define FLOCK_SIZE	       5	 // Size of flock
#define TIME_STEP	       64	 // [ms] Length of time step
#define DELTA_T		    0.064	 // Time step (seconds)
#define AVOIDANCE_THRESHOLD 2100 // Threshold of avoidance mode
#define FOLLOW_THRESHOLD 1300    // Follow wall threshold

// Robots
#define NB_SENSORS	      8	     // Number of distance sensors
#define MIN_SENS          350    // Minimum sensibility value
#define MAX_SENS          4096   // Maximum sensibility value
#define MAX_SPEED         800    // Maximum speed
#define MAX_DIFF (2*MAX_SPEED)   // Maximum difference between wheel speeds
#define MAX_SPEED_WEB      6.28  // Maximum speed webots
#define AXLE_LENGTH 	   0.052 // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS	 0.00628 // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS	 0.0205	 // Wheel radius (meters)


// PSO  -------------------- Choose the number of Sim step -----------------------------------------------------------------
#define SIM_STEPS 600               // number of simulation steps/iterations
#define DATASIZE NB_SENSORS         // Number of elements in particle (2 Neurons with 8 proximity sensors and 5 params for flocking)
#define SCALING_REYNOLD 1000

WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS]; // ps[NB_SENSORS]
WbDeviceTag receiver;		// Handle for the receiver node for flocking
WbDeviceTag emitter;		// Handle for the emitter node for flocking
WbDeviceTag rec_pso;		// Handle for the receiver node for PSO
WbDeviceTag emit_pso;		// Handle for the emitter node for PSO
WbDeviceTag left_encoder;//handler for left encoder of the robot
WbDeviceTag right_encoder;//handler for right encoder of the robot
WbDeviceTag dev_gps; // GPS handler

// Braitenberg for obstacle avoidance
int e_puck_matrix[2*NB_SENSORS] = {17,29,34,10,8,-60,-64,-84,
                        -80,-66,-62,8,10,36,28,18}; // for obstacle avoidance

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
char* robot_name;
int robot_verbose;          // boolean to know for which robot we print

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots "par rapport à moi"
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		        // X, Z, Theta of the current robot
float prev_my_position[3];  		    // X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		        // Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
float migr[2] = {5.5, 0};	      // Migration vector
int avoidance=0;                  // boolean to avoid obstacle
int side=0;                       // side of avoidance (left=0, right=1)
float theta_robots[FLOCK_SIZE];   // Angle theta of the robot



// L.
static double time_step;    // Time step
static measurement_t _meas; // See class in util
static double last_gps_time_s = 0.0f;

// Initial robot positions (x, theta, y) --> in pose (x,y,heading)
pose_t _pose_origin_robot_0 = {-1, 0.0, 0};
pose_t _pose_origin_robot_1 = {-1, 0.1, 0};
pose_t _pose_origin_robot_2 = {-1, -0.1, 0};
pose_t _pose_origin_robot_3 = {-1, 0.2, 0};
pose_t _pose_origin_robot_4 = {-1, -0.2, 0};

static pose_t _pose, _kal_wheel;
static void controller_get_pose_gps();
static void controller_get_gps();
static double controller_get_heading_gps();


/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
      wb_robot_init();
      dev_gps = wb_robot_get_device("gps");
      wb_gps_enable(dev_gps, 1); // Enable GPS every 1000ms <=> 1s
      time_step = wb_robot_get_basic_time_step();

      receiver = wb_robot_get_device("receiver");
      emitter = wb_robot_get_device("emitter");

      //get motors
      left_motor = wb_robot_get_device("left wheel motor");
      right_motor = wb_robot_get_device("right wheel motor");
      wb_motor_set_position(left_motor, INFINITY);
      wb_motor_set_position(right_motor, INFINITY);
      left_encoder = wb_robot_get_device("left wheel sensor");
      right_encoder = wb_robot_get_device("right wheel sensor");
      wb_position_sensor_enable(left_encoder, time_step);
      wb_position_sensor_enable(right_encoder, time_step);


	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name();


	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],TIME_STEP);

	wb_receiver_enable(receiver,TIME_STEP/2);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1

	if(robot_id==0){ // print robot data
    	  robot_verbose=1;
	}

	// Migration
    migr[0] = 0;
    migr[1] = -25;

    // Receiver and emiter PSO
    emit_pso = wb_robot_get_device("emitter_epuck_pso");
    rec_pso = wb_robot_get_device("receiver_epuck_pso");
    wb_receiver_enable(rec_pso, TIME_STEP/2);
}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}


/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) {

	//float theta = my_position[2];
	float dr = (float)msr * SPEED_UNIT_RADS * DELTA_T; //radians
	float dl = (float)msl * SPEED_UNIT_RADS  * DELTA_T;


	int time_step_ = wb_robot_get_basic_time_step();
	double time_now_s = wb_robot_get_time();


    // Position from GPS
    // Position with frame initial point stored in _pose vector
     controller_get_pose_gps();

	 // Update odometry with wheel encoders using pose (GPS derived)
	 compute_kalman_wheels(&_kal_wheel, time_step_, time_now_s, dl,
                                       dr, _pose);

  	 my_position[0] = _kal_wheel.x;
  	 my_position[1] = _kal_wheel.y;
  	 my_position[2] = _kal_wheel.heading;

  	 // Keep orientation within 0, 2pi
	 if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	 if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}


/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter,out,strlen(out)+1);
}


/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void) {
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2];
		double x = message_direction[0];

		theta =	-atan2(y,x);// absolute theta?;why a minus sign?
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));


		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos

		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);

		wb_receiver_next_packet(receiver);
	}
}


// Find the fitness for obstacle avoidance of the passed controller
double simulation_webot(double weights[DATASIZE+1]){

  int msl, msr;			       // Wheel speeds
  double msl_w, msr_w;
  int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
  int i, j;				    // Loop counter
  int ds_value[NB_SENSORS];	// Array for the distance sensor readings
  int max_sens;			    // Store highest sensor value
  double priority;          // non linear weight to prioriterize braitenberg
  msl = 0; msr = 0;
  max_sens = 0;

  // Calcul fitness
  double fit_speed=0;           // Speed aspect of fitness
  double fit_diff=0;            // Speed difference between wheels aspect of fitness
  double fit_sens=0;            // Proximity sensing aspect of fitness
  double fitness;               // Fitness of controller
  double sens_val[NB_SENSORS];  // Average values for each proximity sensor
  for (i=0;i<NB_SENSORS;i++) {  // Initialization
        sens_val[i] = 0.0;
    }

  // Simulation
  for(j=0;j<SIM_STEPS;j++){
    bmsl = 0; bmsr = 0;
    sum_sensors = 0;
    max_sens = 0;

    /* Braitenberg */
    for(i=0;i<NB_SENSORS;i++) {
        ds_value[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
        sum_sensors += ds_value[i]; // Add up sensor values
        max_sens = max_sens>ds_value[i]?max_sens:ds_value[i]; // Check if new highest sensor value

        // Weighted sum of distance sensor values for Braitenburg vehicle
        bmsr += weights[i] * ds_value[i];
        bmsl += (weights[NB_SENSORS-1-i]+2) * ds_value[i];
    }

    // Adapt Braitenberg values (empirical tests)
    bmsl/=MIN_SENS; bmsr/=MIN_SENS;
    bmsl+=66; bmsr+=72;

    /* Send and get information */
    send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

    // Compute self position
    prev_my_position[0] = my_position[0];
    prev_my_position[1] = my_position[1];
    update_self_motion(msl,msr);

    process_received_ping_messages();

    speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
    speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);

    // Apply an initial velocity
    msr=200;
    msl=200;

    priority=2;
    if ((ds_value[0]>AVOIDANCE_THRESHOLD || ds_value[7]>AVOIDANCE_THRESHOLD) && !avoidance){ //Entry1 condition of "avoidance
         avoidance=1;

         // Avoid to the left or right ?
         if(ds_value[7] < ds_value[0]){
           side=0; // left
         }
         else{
           side=1; // right
         }
    }
    else if ((ds_value[6]>AVOIDANCE_THRESHOLD || ds_value[1]>AVOIDANCE_THRESHOLD) && !avoidance){ //Entry2 condition of "avoidance
         avoidance=1;

         // Avoid to the left or right ?
         if(ds_value[6] < ds_value[1]){
           side=0; // left
         }
         else{
           side=1; // right
         }
    }
    else if ((ds_value[5]>AVOIDANCE_THRESHOLD || ds_value[2]>AVOIDANCE_THRESHOLD) && !avoidance){ //Entry2 condition of "avoidance
       avoidance=1;

       // Avoid to the left or right ?
       if(ds_value[5] < ds_value[2]){
         side=0; // left
       }
       else{
         side=1; // right
       }
    }
    if (!avoidance){ // normal mode
        msl -= msl*max_sens/(priority*MAX_SENS);
        msr -= msr*max_sens/(priority*MAX_SENS);
    }
    else{ // avoidance mode
        if(side){ // avoid to the right
            if(ds_value[7]>FOLLOW_THRESHOLD){
                msl = 140;
                msr = 150;
            }
            else{
                msl = 100;
                msr = 150;
            }
        }
        else{ // avoid to the left
            if(ds_value[0]>FOLLOW_THRESHOLD){
                msl = 150;
                msr = 140;
            }
            else{
                msl = 150;
                msr = 100;
            }
        }
    }
    if(avoidance && max_sens<70){ //Exit condition of "avoidance
        avoidance=0;
    }

    // Add Braitenberg
    msl += bmsl;
    msr += bmsr;
    limit(&msl,MAX_SPEED);
    limit(&msr,MAX_SPEED);

    // Set speed
    msl_w = msl*MAX_SPEED_WEB/(MAX_SPEED+1);
    msr_w = msr*MAX_SPEED_WEB/(MAX_SPEED+1);

    wb_motor_set_velocity(left_motor, msl_w);
    wb_motor_set_velocity(right_motor, msr_w);

    // Continue one step
    wb_robot_step(TIME_STEP);

    // Get current fitness value
    // Average speed
    fit_speed += (fabs(msl) + fabs(msr))/(2.0*MAX_SPEED);

    // Difference in speed
    fit_diff += fabs(msl - msr)/MAX_DIFF;

    // Sensor values
    for (i=0;i<NB_SENSORS;i++) {
        sens_val[i] += ds_value[i]/MAX_SENS;
    }
  }
  // Find most active sensor
  for (i=0;i<NB_SENSORS;i++) {
      if (sens_val[i] > fit_sens) fit_sens = sens_val[i];
  }
  // Average values over all steps
  fit_speed /= SIM_STEPS;
  fit_diff /= SIM_STEPS;
  fit_sens /= SIM_STEPS;

  // Better fitness should be higher
  fitness = fit_speed*(1.0 - sqrt(fit_diff))*(1.0 - fit_sens);

  return fitness;
}


/*
 * Main function : receives the weight from supervisor, send them to the simulation and send back the fitness to the supervisor when the simulation end
*/
int main() {
    // Init
    double end_sim[255];
    double *new_weights;
    double fitness=-1;
    reset();
    wb_motor_set_velocity(left_motor, 0);  // Initialize robot velocity to zero
    wb_motor_set_velocity(right_motor, 0); // Initialize robot velocity to zero

    // Forever Loop
    while (1) {
        // Wait for data
        while (wb_receiver_get_queue_length(rec_pso) == 0) {
            wb_robot_step(TIME_STEP);
        }

        // Print weight
        if( 1 && robot_verbose){printf("************************ Weight ***************************\n");}
        new_weights = (double *)wb_receiver_get_data(rec_pso);
        if( 1 && robot_verbose){printf("Robot %d : %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n         %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf\n----------------------------\n",robot_id, new_weights[0], new_weights[1],new_weights[2], new_weights[3], new_weights[4], new_weights[5],new_weights[6], new_weights[7],new_weights[7]+2, new_weights[6]+2, new_weights[5]+2, new_weights[4]+2, new_weights[3]+2, new_weights[2]+2, new_weights[1]+2, new_weights[0]+2);}

        // Update initial position
        my_position[0]=0;
        my_position[1]=0;
        my_position[2]=0;
        prev_my_position[0]=0;
        prev_my_position[1]=0;
        prev_my_position[2]=0;
        _pose_origin_robot_0.y = new_weights[DATASIZE];
        _pose_origin_robot_1.y = new_weights[DATASIZE];
        _pose_origin_robot_2.y = new_weights[DATASIZE];
        _pose_origin_robot_3.y = new_weights[DATASIZE];
        _pose_origin_robot_4.y = new_weights[DATASIZE];
        controller_get_pose_gps();
        kal_reset();

        // Run simulation
        fitness = simulation_webot(new_weights);

        end_sim[0]=fitness;
        wb_emitter_send(emit_pso,(void *)end_sim,sizeof(double));
        wb_receiver_next_packet(rec_pso);

        if( 1 && robot_verbose){printf("********************************************************\n");}
    }
    return 0;
}


// Functions used for Kalman
void controller_get_pose_gps() {
    double time_now_s = wb_robot_get_time();

    if (time_now_s - last_gps_time_s > 1.0f) {
        controller_get_gps();
        last_gps_time_s = time_now_s;
        // This is for the robot which is starting
        if (robot_id_u == 0) {
        _pose.x = _meas.gps[0] - _pose_origin_robot_0.x;
        _pose.y = -(_meas.gps[2] - _pose_origin_robot_0.y);
        _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_0.heading;
        }
        else if (robot_id_u == 1) {
        _pose.x = _meas.gps[0] - _pose_origin_robot_1.x;
        _pose.y = -(_meas.gps[2] - _pose_origin_robot_1.y);
        _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_1.heading;
        }
        else if (robot_id_u == 2) {
        _pose.x = _meas.gps[0] - _pose_origin_robot_2.x;
        _pose.y = -(_meas.gps[2] - _pose_origin_robot_2.y);
        _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_2.heading;
        }
         else if (robot_id_u == 3) {
        _pose.x = _meas.gps[0] - _pose_origin_robot_3.x;
        _pose.y = -(_meas.gps[2] - _pose_origin_robot_3.y);
        _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_3.heading;
        }
        else if (robot_id_u == 4){
        _pose.x = _meas.gps[0] - _pose_origin_robot_4.x;
        _pose.y = -(_meas.gps[2] - _pose_origin_robot_4.y);
        _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_4.heading;
        }
    }
}


/**
 *
 * @brief Get the GPS measurements for position of robots
 */
void controller_get_gps() {
    /// Stores in memory at address of _meas.prev_gps; the data of _meas.gps
    memcpy(_meas.prev_gps, _meas.gps, sizeof(_meas.gps));

    // Get position
    const double *gps_position = wb_gps_get_values(dev_gps);

    // Stores in memory at address of _meas.gps, the data of computed gps_position
    memcpy(_meas.gps, gps_position, sizeof(_meas.gps));


}


/**
 * @brief      Compute the heading (orientation) of the robot based on the gps position values.
 *
 * @return     return the computed angle in radians
 */
double controller_get_heading_gps() {
    // Orientation of the robot
    double delta_x = _meas.gps[0] - _meas.prev_gps[0];
    double delta_y = _meas.gps[2] - _meas.prev_gps[2];

    // Compute the heading of the robot
    double heading = atan2(delta_y, delta_x);

    return heading;
}


/**
 * @brief      Read the encoders values from the sensors
 */
void controller_get_encoder() {
    // Store previous value of the left encoder
    _meas.prev_left_enc = _meas.left_enc;
    _meas.left_enc = wb_position_sensor_get_value(left_encoder);
    // If nan // That is you are at the first itteration
    if (isnan(_meas.left_enc)) {
    _meas.left_enc = 0.0;
    ;}

    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;
    _meas.right_enc = wb_position_sensor_get_value(right_encoder);
        if (isnan(_meas.right_enc)) {
    _meas.right_enc = 0.0;
    ;}
}


