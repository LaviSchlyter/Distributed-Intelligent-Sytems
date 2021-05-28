/*****************************************************************************/
/* File:         crossing_leader.c                                           */
/* Version:      1.0                                                         */
/* Date:         06-Jun-21                                                   */
/* Description:  Formation with relative positions in a world with           */
/*               two teams of robots crossing : leader controller            */
/*                                                                           */
/* Author:      06-Jun-21 by Tifanny Portela                                 */
/*****************************************************************************/



/**
Analysis of the frame and choices:
- the world has its x = 0; all the way into the right corner
Robot 5 (Left):It's original position w.r.t world: PL= {-1.91, 0.0, 0};
For the right robot (0); orginally at positon PR = {-0.06, 0.0, 0};
Right now I am taking the pose with the respect to each robot individually;
this means that pose_x will always be positive (except if going backwards...)


Possible mistakes:

#1)  If nan value, I put zero. 
Problem --> The difference between wheel encoder_r and wheel encoder_prev_r
becomes huge ! The position is thus greatly over-estimated 
- quick fix (though debatable). 
In odometry.c:

QF1 = if (Aleft_enc <0.30 & Aright_enc < 0.3) --> then update the posiiton else not (this value is only superior in the second time step)
Problem-->Small underestimation. Because I consider it does not move whilst it does 

#2) For now Kalman also implements QF1, else it looks quite good :) 



---------------------- TO DO -----------------------------

- Implmenent migration urge [LAVI] 

Problem 25.05

- Implemented but does not go in desired direction ! AT ALL 


The idea is that the left group goes to right origin and vice versa
- If robot 5 then go to _pose_origin_R
- If robot 0 then go to _pose_origin_L

**/




#include <stdio.h>
#include <math.h>
#include <string.h>
#include <float.h>


#include "../localization_controller_test/utils.h"
#include "../localization_controller_test/odometry.h"
#include "../localization_controller_test/kalman.h"


#include <webots/robot.h>
#include <webots/motor.h>
<<<<<<< HEAD
#include <webots/gps.h>
#include <webots/position_sensor.h>
/*Webots 2018b*/
=======
>>>>>>> 6da2e62108a5093d4f2afc6224c5494d13e9c3ed
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS	      8	      // Number of distance sensors
<<<<<<< HEAD
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define TIME_STEP	  64	  // [ms] Length of time step
#define AVOIDANCE_THRESH    1000  // Threshold above which we enter obstacle avoidance
#define WHEEL_AXIS        0.057        // Distance between the two wheels in meter
#define WHEEL_RADIUS            0.020        // Radius of the wheel in meter
=======
#define MIN_SENS           350      // Minimum sensibility value
#define MAX_SPEED          800      // Maximum speed
#define MAX_SPEED_WEB     6.28    // Maximum speed webots
#define TIME_STEP	      64	      // Length of time step in [ms] 
#define AVOIDANCE_THRESH  1000   // Threshold above which we enter obstacle avoidance
>>>>>>> 6da2e62108a5093d4f2afc6224c5494d13e9c3ed

// two leaders
#define LEADER1_ID 0  // leader ID of robots_ID_group1
#define LEADER2_ID 5  // leader ID of robots_ID_group2

//States of FSM
#define AVOIDANCE 0
#define MIGRATION 1
#define MIGRATION_WEIGHT  1   // Wheight of attraction towards the common goal. default 0.01/10

/*VARIABLES*/
// L.
double migrLeft[2] = {1.85, 0.5};
//double migrLeft[2] = {-1.91, 0.0};
double migrRight[2] = {1.85, 0.5};
//double migrRight[2] = {-0.06, 0.0};
static pose_t _pose, _odo_enc, _kal_wheel;

// Initial robot position
static pose_t _pose_origin_R = {-0.06, 0.0, M_PI};
static pose_t _pose_origin_L = {-1.91, 0.0, 0};
static double speed[2];                 // Speed calculated for migration




<<<<<<< HEAD
/*Webots 2018b*/
WbDeviceTag dev_gps; // GPS handler 
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag left_encoder;//handler for left encoder of the robot
WbDeviceTag right_encoder;//handler for right encoder of the robot
/*Webots 2018b*/
=======
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
>>>>>>> 6da2e62108a5093d4f2afc6224c5494d13e9c3ed

int e_puck_matrix[2*NB_SENSORS] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;	    // Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id_u;	            // Unique robot ID
char* robot_name;

float range_to_other_leader;     // measured range to the leader of the other team
float bearing_to_other_leader;   // measured bearing to the leader of the other team

int fsm_state;               // leader's state of tehe FSM (avoidance or migration)

static double time_step;                  // Time step
static measurement_t _meas; // See class in util
static double last_gps_time_s = 0.0f;


/*FUNCTIONS*/
// Make static to limit its scope

static void controller_get_pose_gps();

static void controller_get_gps();

static double controller_get_heading_gps();

static void controller_get_encoder();







/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	
	
	dev_gps = wb_robot_get_device("gps");
        wb_gps_enable(dev_gps, 1); // Enable GPS every 1000ms <=> 1s
	time_step = wb_robot_get_basic_time_step();
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	//get encoders L.
	
	left_encoder = wb_robot_get_device("left wheel sensor");
	right_encoder = wb_robot_get_device("right wheel sensor");
	wb_position_sensor_enable(left_encoder, time_step);
        wb_position_sensor_enable(right_encoder, time_step);
        
	
	//get motors
	left_motor = wb_robot_get_device("left wheel motor");
        right_motor = wb_robot_get_device("right wheel motor");
        wb_motor_set_position(left_motor, INFINITY);
        wb_motor_set_position(right_motor, INFINITY);
	
	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name(); 
	

	for(i=0;i<NB_SENSORS;i++){
		wb_distance_sensor_enable(ds[i],64);
       }

	wb_receiver_enable(receiver,64);

	//Reading the robot's name.
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
  
    printf("Reset: robot %d\n",robot_id_u);
}


/*
 * Keep given int numbers within interval {-limit, limit} and rescale both numbers to keep their relative differences
 * If above limit, set biggest number to limit and the smallest number to (limit - their initial difference)
 */
void limit_and_rescale(int *number1, int *number2, int limit) {

    int delta;
    
    if ((*number1 > 0) && (*number2 > 0)){
    
        if (*number1 > *number2){
         
            delta = *number1 - *number2;
            if (*number1 > limit){
                *number1 = limit;
                *number2 = limit-delta;
            }
        
        }else{
         
            delta = *number2 - *number1;
            if (*number2 > limit){
                *number2 = limit;
                *number1 = limit-delta;
            }
        }
    }
    else if ((*number1 > 0) && (*number2 < 0)){
    
        delta = *number1 - *number2;
    
        if ((*number1 >= limit) && (*number2 <= -limit)){
          *number1 = limit;
          *number2 = -limit;
        }
        else if((*number1 >= limit) && (*number2 > -limit)){
          *number1 = limit;
          *number2 = limit-delta;
        }
        else if((*number1 < limit) && (*number2 <= -limit)){
          *number1 = -limit+delta;
          *number2 = -limit;
        }
          
    }
    else if ((*number1 < 0) && (*number2 > 0)){
    
        delta = *number2 - *number1;
    
        if ((*number1 <= -limit) && (*number2 >= limit)){
          *number1 = -limit;
          *number2 = limit;
        }
        else if((*number1 > -limit) && (*number2 >= limit)){
          *number1 = limit-delta;
          *number2 = limit;
        }
        else if((*number1 <= -limit) && (*number2 < limit)){
          *number1 = -limit;
          *number2 = -limit+delta;
        }
      
    }
    else if ((*number1 < 0) && (*number2 < 0)){
    
        if (*number1 < *number2){
         
            delta = *number2 - *number1;
            if (*number1 < -limit){
                *number1 = -limit;
                *number2 = -limit+delta;
            }
        
        }else{
         
            delta = *number1 - *number2;
            if (*number2 < -limit){
                *number1 = -limit+delta;
                *number2 = -limit;
            }
        }
    }
}


/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) 
{
	// Compute wanted position from Reynold's speed and current location
        //printf("SPEED ENTERIN")
	//float x = speed[0]*cosf(controller_get_heading_gps()) + speed[1]*sinf(controller_get_heading_gps()); // x in robot coordinates
	//printf("speed[0] = %g, cosf = %lf, speed[1] = %g, sinf = %lf\n\n",speed[0], cos(controller_get_heading_gps()),speed[1],sinf(controller_get_heading_gps()));
	//float z = -speed[0]*sinf(controller_get_heading_gps()) + speed[1]*cosf(controller_get_heading_gps()); // z in robot coordinates
	float x = speed[0]*cosf(_kal_wheel.heading) + speed[1]*sinf(_kal_wheel.heading); // x in robot coordinates
	printf("speed[0] = %g, cosf = %lf, speed[1] = %g, sinf = %lf\n\n",speed[0], cos(_kal_wheel.heading),speed[1],sinf(_kal_wheel.heading));
	float z = -speed[0]*sinf(_kal_wheel.heading) + speed[1]*cosf(_kal_wheel.heading); // z in robot coordinate
        
        printf("x = %f ; z = %f\n", x, z);
	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = atan2(z, x);	  // Orientation of the wanted position
	printf("bearing to wanted = %g\n", RAD2DEG(bearing));
	printf("range of interest = %g\n", range);
	printf("Heading GPS = %g\n", RAD2DEG(controller_get_heading_gps()));
	printf("Kalman heading = %g\n", RAD2DEG(_kal_wheel.heading));
	
	// Compute forward control= 3.22526e-319
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;
	
	// Convert to wheel speeds!
	*msl = (u - WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + WHEEL_AXIS*w/2.0) * (1000.0 / WHEEL_RADIUS);
	printf("msl = %lf\n", *msl);
	printf("msr = %lf\n", *msr);

        limit_and_rescale(msl, msr, MAX_SPEED);
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
    double theta, bearing, range;
    char *inbuffer;	// Buffer for the receiver node
    int friend_robot_id;
    while (wb_receiver_get_queue_length(receiver) > 0) {
    
          inbuffer = (char*) wb_receiver_get_data(receiver);
          message_direction = wb_receiver_get_emitter_direction(receiver);
          message_rssi = wb_receiver_get_signal_strength(receiver);
          double z = message_direction[2];
          double x = message_direction[0];
          
          theta = -atan2(z,x); 
          bearing = theta - M_PI/2; // find the relative theta;
          range = sqrt((1/message_rssi));
          
          if (bearing < 0){// keep bearing between 0 and 2pi
      	       bearing = bearing + 2*M_PI;
          }
          
          friend_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message.

          
          // save the measured range and bearing if the sender is the other leader
          if ((friend_robot_id == LEADER1_ID) || (friend_robot_id == LEADER2_ID)){

             range_to_other_leader = range;
             bearing_to_other_leader = bearing;
             
             //printf("Robot ID %d, my range %f and bearing %f to the other leader %d\n ", robot_id_u, range, bearing, friend_robot_id);
          }
          
           wb_receiver_next_packet(receiver);
    }
}


// the main function
int main(){ 
        
           
        int msl = 0;             // Left wheel speed
        int msr = 0;            // Right wheel speed
        float msl_w, msr_w;
        int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
        int i;                          // Loop counter
        int ds_value[NB_SENSORS];       // Array for the distance sensor readings
        int max_sens;                   // Store highest sensor value

        wb_robot_init();
        reset();                    // Resetting the robot
        odo_reset(time_step);
        
  // initial state: migartion
        fsm_state = MIGRATION;
        

        // Forever
        for(;;){
        
            
            bmsl = 0; bmsr = 0;
            sum_sensors = 0;
            max_sens = 0;

              // Braitenberg
              for(i = 0; i < NB_SENSORS; i++) {
                      ds_value[i] = wb_distance_sensor_get_value(ds[i]); //Read sensor values
                      sum_sensors += ds_value[i]; // Add up sensor values
                      max_sens = max_sens>ds_value[i]?max_sens:ds_value[i]; // Check if new highest sensor value
                            // Weighted sum of distance sensor values for Braitenburg vehicle
                      bmsr += e_puck_matrix[i] * ds_value[i];
                      bmsl += e_puck_matrix[i + NB_SENSORS] * ds_value[i];
              }
              
              // Adapt Braitenberg values (empirical tests)
              bmsl/=MIN_SENS; bmsr/=MIN_SENS;
              bmsl+=66; bmsr+=72;
              
              send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
              process_received_ping_messages();
              
<<<<<<< HEAD
              
=======
              //printf("LEADER %d -----> ds_value[0] %d, ds_value[7] %d \n", robot_id_u, ds_value[0],ds_value[7]);
>>>>>>> 6da2e62108a5093d4f2afc6224c5494d13e9c3ed
             
              // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
              if ((ds_value[0] > AVOIDANCE_THRESH ||
                   ds_value[7] > AVOIDANCE_THRESH ||
                   ds_value[6] > AVOIDANCE_THRESH ||
                   ds_value[1] > AVOIDANCE_THRESH) && (fsm_state == MIGRATION)){
                   
                   printf("LEADER %d avoidance \n", robot_id_u);
              
                  fsm_state = AVOIDANCE;
              }
            
            
            if (fsm_state == MIGRATION){ //formation state --- do only formation
            
              
              printf("------------------------------- ROBOT_ID = %d--------------------------   \n  ", robot_id_u);
             printf("---------Pose robot x= %lf, y=%lf, heading =%lf\n", _pose.x, _pose.y,RAD2DEG(_pose.heading));
              // Get Position using Kalman
              time_step = wb_robot_get_basic_time_step();
              // Position from GPS
              // Position with frame initial point stored in _pose vector
              controller_get_pose_gps();
  
              // Get the encoder values (wheel motor values)
              controller_get_encoder();
           
              // Update gps measurements
              controller_get_gps();
           
              /// Compute position from wheel encoders
              odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc,
                                  _meas.right_enc - _meas.prev_right_enc);
              
  
            
            
              
              int time_step_ = wb_robot_get_basic_time_step();
  
  
              double time_now_s = wb_robot_get_time();
              

  
                  // Kalman with wheel encoders
             compute_kalman_wheels(&_kal_wheel, time_step_, time_now_s, _meas.left_enc - _meas.prev_left_enc,
                                        _meas.right_enc - _meas.prev_right_enc, _pose);
                                        
                                        
               // This is the leader robot wanting to go left 
                
                if (robot_id_u == 0) {
                        printf("Robot 0: Kalman_x = %g, Kalman_y = %g\n", _kal_wheel.x, _kal_wheel.y); 
                        
                        //double tmp_x = (migrLeft[0]-_meas.gps[0]);
                        //double tmp_z = (migrLeft[1]-_meas.gps[2]);
                        double tmp_x = (migrLeft[0]-_kal_wheel.x);
                        double tmp_z = (migrLeft[1]-_kal_wheel.y);
                        printf("tmp_x = %g, tmp_z = %g\n", tmp_x, tmp_z);
                       speed[0] = tmp_x * MIGRATION_WEIGHT;
                  	speed[1] = tmp_z * MIGRATION_WEIGHT;
                  	printf("ID = 0:speed 0 = %g speed 1 = %g\n", speed[0], speed[1]);
                
                }
                else if (robot_id_u == 5) {
                printf("Robot 5: Kalman_x = %g, Kalman_y = %g\n", _kal_wheel.x, _kal_wheel.y); 
                    
                    //double tmp_x = (migrRight[0]-_meas.gps[0]);
                    //double tmp_z = (migrRight[1]-_meas.gps[2]);
                    double tmp_x = (migrRight[0]-_kal_wheel.x);
                    double tmp_z = (migrRight[1]-_kal_wheel.y);
                    
                    speed[0] = tmp_x * MIGRATION_WEIGHT;
                    speed[1] = tmp_z * MIGRATION_WEIGHT;
      		    
      		    
      		    
                }
                //printf("mirgLeftx = %g,mirgLefty = %g, meas_gps = %g\n", migrLeft[0],migrLeft[1], _meas.gps[0]);
                //printf("migrRightx = %g,migrRighty = %g, meas_gps = %g\n", migrRight[0],migrRight[1], _meas.gps[0]);
                
            
                
              compute_wheel_speeds(&msl, &msr);
              
              
              //msl = 200;
              //msr = 200;
              
                
         

            }
            else{ 
            //avoidance state --- do only braitenberg
                
                msl = bmsl;
                msr = bmsr;
            }
            
            limit_and_rescale(&msl, &msr, MAX_SPEED);
            
            // Set speed
            msl_w = msl*MAX_SPEED_WEB/(MAX_SPEED+1);
            msr_w = msr*MAX_SPEED_WEB/(MAX_SPEED+1);
            
            wb_motor_set_velocity(left_motor, msl_w);
            wb_motor_set_velocity(right_motor, msr_w);
               
            //Condition to exit avoidance state
            if((fsm_state == AVOIDANCE) && (max_sens < 70)){ //Exit condition of "avoidance
                fsm_state = MIGRATION;
            }

               
            // Continue one step --> change robot
            wb_robot_step(TIME_STEP);
         }
 }
 

// Functions used for Kalman

void controller_get_pose_gps() {

    double time_now_s = wb_robot_get_time();

    //if (time_now_s - last_gps_time_s > 0.50f) {
    if (time_now_s - last_gps_time_s > 1.0f) {

        last_gps_time_s = time_now_s;
        // This is for the robot which is starting 
        if (robot_id_u == 0) {

        _pose.x = (_meas.gps[0] - _pose_origin_R.x)*cosf( _pose_origin_R.heading);

        _pose.y = (_meas.gps[2] - _pose_origin_R.y);

        _pose.heading = -controller_get_heading_gps() + _pose_origin_R.heading;
        }
        else if (robot_id_u == 5){
        _pose.x = _meas.gps[0] - _pose_origin_L.x;

        _pose.y = -(_meas.gps[2] - _pose_origin_L.y);

        _pose.heading = -controller_get_heading_gps() + _pose_origin_L.heading;
        printf("GPS heading = %lf\n", RAD2DEG(-controller_get_heading_gps()));
        printf("Robot5 heading = %lf\n", RAD2DEG(_pose.heading));
        
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
    
    printf("GPSx = %g, GPSy = %g\n", _meas.gps[0], _meas.gps[2]);

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

    _meas.gps_heading[0] = heading;
    
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
    printf("Left encoder = %g\n", _meas.left_enc);
    
    
    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;
    

    _meas.right_enc = wb_position_sensor_get_value(right_encoder);
    
        if (isnan(_meas.right_enc)) {
    _meas.right_enc = 0.0;
    
    ;}
    printf("Riht encoder = %g\n", _meas.right_enc);
}


