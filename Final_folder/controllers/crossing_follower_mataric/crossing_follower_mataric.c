/**************************************************************************************************************************/
/* File:         crossing_follower_mataric.c                                 */
/* Version:      1.0                                                         */
/* Date:         06-Jun-21                                                   */
/* Description:  Formation with relative positions in a world where two      */
/*               teams of robots are crossing: follower controller (mataric) */
/*                                                                           */
/* Author:       Tifanny Portela                                             */
/**************************************************************************************************************************/

/* Tunable parameters:                                                                                                      */
/* 1) This controller works for a FLOCK_SIZE of size 2,3,4 or 5 (change the value accordingly at line 30)                   */
/* 2) The controller type can be: (change the type accordingly at line 36)                                                  */
/*     - Proportionnal (P)                                                                                                  */
/*     - Proportional + integral (PI)                                                                                       */
/*     - Non linear version of a PI (NPI)                                                                                   */

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

// ------------------------- Adapt the flock size  ---------------------------
#define FLOCK_SIZE            5     // Size of flock (2,3,4 or 5) of one group for Mataric controller

// ------------------------- Choose your controller  -------------------------
#define P   0
#define PI  1
#define NPI 2
#define CONTROLLER_TYPE PI

// ------------------------- Choose your world  -------------------------
#define CROSSING 0
#define TEST_CROSSING 1
#define WORLD TEST_CROSSING

#define NB_SENSORS	       8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SPEED         800     // Maximum speed
#define MAX_SPEED_WEB     6.28    // Maximum speed webots
#define MAX_FLOCK_SIZE      5     // Maximum size of flock
#define TIME_STEP	       64	  // Length of time step in [ms]

#define AVOIDANCE_THRESH   1800  // Threshold above which we enter obstacle avoidance
#define AXLE_LENGTH 	   0.052	// Distance between wheels of robot (meters)
#define WHEEL_RADIUS	   0.0205	// Wheel radius (meters)
#define DELTA_T			   0.064	// Timestep (seconds)

//Use PSO-optimized weights; if 0, the code will use empirical values
#define PSO 0
#if PSO
//Matrix of Braitenberg sensor weights for obstacle avoidance
int e_puck_matrix[2*NB_SENSORS] = {-2.75, 46.2, 149, 193.4, 168, -31,-164, -16,
                                   -16, -164, -31, 168, 193.4, 149, 46.2, -2.75}; 
#define FORMATION_THRESH    180    // Threshold under which we enter formation state
#else
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,
                        -72,-58,-36,8,10,36,28,18}; // empirical values
#define FORMATION_THRESH    70    // Threshold under which we enter formation state
#endif

//boolean values 
#define TRUE 1
#define FALSE 0

// two leaders
#define LEADER1_ID 0  // leader ID of robots_ID_group1
#define LEADER2_ID 5  // leader ID of robots_ID_group2

//States of FSM
#define AVOIDANCE 0
#define FORMATION 1

WbDeviceTag left_motor;      //handler for left wheel of the robot
WbDeviceTag right_motor;     //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];	 //Handle for the infrared distance sensors
WbDeviceTag receiver;	     //Handle for the receiver node
WbDeviceTag emitter;		 //Handle for the emitter node

int fsm_state;                // finite state machine's state
int robot_id_u;	              // unique robot ID
char* robot_name;

// for PI controller : using tustin's approx
float integrator;
float prev_delta_bearing;

float measured_range_bearing[2]; // measured range and bearing to your friend
float target_range = 0.1414;
float target_bearing;
float target_bearing_followers[MAX_FLOCK_SIZE - 1] = { M_PI/4 , 7*M_PI/4 , M_PI/4 , 7*M_PI/4 };

/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	wb_robot_init();
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");
	
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
		wb_distance_sensor_enable(ds[i],TIME_STEP);
       }

	wb_receiver_enable(receiver,TIME_STEP);

	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
 
}

/*
* Keep numbers within interval {-limit, limit} and rescale both numbers to keep their relative difference unchanged.
* If the number is above the limit, set the biggest number to the limit and the smallest number to (limit - their difference)
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
        }
        else{
            delta = *number1 - *number2;
            if (*number2 < -limit){
                *number1 = -limit+delta;
                *number2 = -limit;
            }
        }
    }
}

/*
 * Check if the robot_ID is in group 1 (return True if that's the case)
 */
int robotID_in_group1(robot_id_u){

   bool is_in_group1 = FALSE;
   
   if (robot_id_u < LEADER2_ID){
      is_in_group1 = TRUE;
   }
   
   return is_in_group1;
}

/*
 * Returns the corresponding robot ID in group 1 of the current robot for the world crossing
*/
int corresponding_robot_id_in_group1(){

    int corresponding_ID;
    
    if (robot_id_u < MAX_FLOCK_SIZE){
        corresponding_ID = robot_id_u;
    }
    else{
        if (robot_id_u == 5){
            corresponding_ID = 0;
        }
        else if (robot_id_u == 6){
            corresponding_ID = 2;
        }
        else if (robot_id_u == 7){
            corresponding_ID = 1;
        }
        else if (robot_id_u == 8){
            corresponding_ID = 4;
        }
        else {
            corresponding_ID = 3;
        }
    }

    return corresponding_ID;
}

/*
 * Returns the corresponding robot ID in group 1 of the current robot for the world test_crossing
*/
int corresponding_robot_id_in_group1t(){

    int corresponding_ID;
    
    if (robot_id_u < MAX_FLOCK_SIZE){
        corresponding_ID = robot_id_u;
    }
    else{
        if (robot_id_u == 5){
            corresponding_ID = 0;
        }
        else if (robot_id_u == 6){
            corresponding_ID = 1;
        }
        else if (robot_id_u == 7){
            corresponding_ID = 2;
        }
        else if (robot_id_u == 8){
            corresponding_ID = 3;
        }
        else {
            corresponding_ID = 4;
        }
    }

    return corresponding_ID;
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
 * Processing all the received ping messages, and calculate range and bearing to the other robots
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
          
          if (WORLD == CROSSING){
		  // group1
		  if (robotID_in_group1(robot_id_u)){
      		    if ((FLOCK_SIZE >= 2) &&(friend_robot_id == 0) && (robot_id_u == 1)){
            	        measured_range_bearing[0] = range;
            	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 3) &&(friend_robot_id == 0) && (robot_id_u == 2)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 4) && (friend_robot_id == 2) && (robot_id_u == 4)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 5) &&(friend_robot_id == 1) && (robot_id_u == 3)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                   }
		   }
		   else { // group2
      		    if ((FLOCK_SIZE >= 2) && (friend_robot_id == 5) && (robot_id_u == 7)){
            	        measured_range_bearing[0] = range;
            	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 3) && (friend_robot_id == 5) && (robot_id_u == 6)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 4) && (friend_robot_id == 6) && (robot_id_u == 8)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 5) && (friend_robot_id == 7) && (robot_id_u == 9)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
		 }
	      }
	      else {
    	          		  // group1
		  if (robotID_in_group1(robot_id_u)){
      		    if ((FLOCK_SIZE >= 2) &&(friend_robot_id == 0) && (robot_id_u == 1)){
            	        measured_range_bearing[0] = range;
            	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 3) &&(friend_robot_id == 0) && (robot_id_u == 2)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 4) && (friend_robot_id == 2) && (robot_id_u == 4)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 5) &&(friend_robot_id == 1) && (robot_id_u == 3)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                   }
		   }
		   else { // group2
      		    if ((FLOCK_SIZE >= 2) && (friend_robot_id == 5) && (robot_id_u == 6)){
            	        measured_range_bearing[0] = range;
            	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 3) && (friend_robot_id == 5) && (robot_id_u == 7)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 4) && (friend_robot_id == 7) && (robot_id_u == 9)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
                  else if ((FLOCK_SIZE >= 5) && (friend_robot_id == 6) && (robot_id_u == 8)){
              	        measured_range_bearing[0] = range;
              	        measured_range_bearing[1] = bearing;
                  }
		 }
	      
	      }

		    
		    
           wb_receiver_next_packet(receiver);
    }
}

/*
 * Calculates the weight of the integral part of the controller (allowing the intagrator to 
 * discharge when the leader and the followers are not aligned)
*/
float sigmoid(float delta_bearing){
  
  float slope = 10.0;
  float weight_coeff = 5.0;
  float alpha0 = M_PI/4.0;
  float gamma = alpha0 + 1.0/slope*logf(weight_coeff - 1.0);
  
  if (delta_bearing > M_PI){
      delta_bearing = delta_bearing - 2*M_PI;  // between -pi and pi to use sigmoid 
  }
  float sigmoid = 1.0 - weight_coeff*(1.0/(1.0 + exp(-slope*(delta_bearing - gamma))));
  
  return sigmoid;
}


/*
 * Given the measured range and bearing of your friend, determine the control commands 
 * (u = linear velocity, omega = angular velocity) to match your target range and bearing
*/

void range_bearing_to_command(int *msl, int *msr){

    float Ku = 0.2;  // Forward control coefficient 0.2
    float Kw = 0.4;  // Rotational control coefficient 0.5
    float Ki = 0.001;  // integrator coeff 0.01
    float delta_bearing = 0.0;
    float delta_range = 0.0;
    float u,w;
    
   
    delta_range = measured_range_bearing[0] - target_range;
    delta_bearing = measured_range_bearing[1] - target_bearing;
    
    if (CONTROLLER_TYPE == P){
          u = Ku*delta_range*cosf(delta_bearing);// Compute forward control
          w = Kw*delta_bearing;// Compute rotational control
    }
    else if (CONTROLLER_TYPE == PI){
          integrator = integrator + DELTA_T/2.0*(prev_delta_bearing + delta_bearing);
          u = Ku*delta_range*cosf(delta_bearing) + Ki*integrator;// Compute forward control
          w = Kw*delta_bearing;// Compute rotational control
          prev_delta_bearing = delta_bearing;
    }
    else if (CONTROLLER_TYPE == NPI){
          integrator = integrator + DELTA_T/2.0*(prev_delta_bearing*sigmoid(prev_delta_bearing) + delta_bearing*sigmoid(delta_bearing));
          u = Ku*delta_range*cosf(delta_bearing) + Ki*integrator;// Compute forward control
          w = Kw*delta_bearing;// Compute rotational control
          prev_delta_bearing = delta_bearing;
    }
    
    *msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    limit_and_rescale(msl, msr, MAX_SPEED);
}

        
// the main function
int main(){
       
    int msl = 0;                    // Left wheel speed
    int msr = 0;			        // Right wheel speed
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int i;                          // Loop counter
    int ds_value[NB_SENSORS];       // Array for the distance sensor readings
    int max_sens;                   // Store highest sensor value
    int corresponding_robot_id;     // Corresponding robot ID in group 1
   
    reset();	
    if (WORLD == CROSSING){            
      corresponding_robot_id = corresponding_robot_id_in_group1();
    }
    else{
      corresponding_robot_id = corresponding_robot_id_in_group1t();
    }
    
    target_bearing = target_bearing_followers[corresponding_robot_id - 1]; // Set the bearing target for the current robot
    fsm_state = FORMATION; // initial FSM's state: formation

	for(;;){ // Forever
        bmsl = 0; bmsr = 0;
        sum_sensors = 0;
        max_sens = 0;
        
        // Braitenberg
        for(i = 0; i < NB_SENSORS; i++) {
            ds_value[i] = wb_distance_sensor_get_value(ds[i]);
          	sum_sensors += ds_value[i];
          	max_sens = max_sens>ds_value[i]?max_sens:ds_value[i]; // Check if new highest sensor value
            // Weighted sum of distance sensor values for Braitenburg vehicle
          	bmsr += e_puck_matrix[i] * ds_value[i];
          	bmsl += e_puck_matrix[i + NB_SENSORS] * ds_value[i];
         }
          				
          // Adapt Braitenberg values (empirical tests)
          bmsl/=MIN_SENS; bmsr/=MIN_SENS;
          bmsl+=66; bmsr+=72;
    
          /* Send and get information */
          send_ping();
          process_received_ping_messages();
    
          // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
          if ((ds_value[0] > AVOIDANCE_THRESH || ds_value[7] > AVOIDANCE_THRESH ||
               ds_value[6] > AVOIDANCE_THRESH || ds_value[1] > AVOIDANCE_THRESH) && (fsm_state == FORMATION)){
              fsm_state = AVOIDANCE;
          }
           
          if (fsm_state == FORMATION){ //formation state --- do only formation
               range_bearing_to_command(&msl, &msr); //formation command
          }
          else{ //avoidance state --- do only braitenberg
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
          if((fsm_state == AVOIDANCE) && (max_sens < FORMATION_THRESH)){
               fsm_state = FORMATION;
          }

          // Continue one step
          wb_robot_step(TIME_STEP);
	 }
}  
  

  

