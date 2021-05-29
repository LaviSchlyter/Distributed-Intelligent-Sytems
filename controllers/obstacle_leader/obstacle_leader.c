/*****************************************************************************/
/* File:         obstacle_leader.c                                           */
/* Version:      1.0                                                         */
/* Date:         06-Jun-21                                                   */
/* Description:  Formation with relative positions in a world with           */
/*               obstacles : leader controller                               */
/*                                                                           */
/* Author:      06-Jun-21 by Tifanny Portela                                 */
/*****************************************************************************/


#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS	      8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define TIME_STEP	  64	  // [ms] Length of time step
#define AVOIDANCE_THRESH    1800  // Threshold above which we enter obstacle avoidance

//States of FSM
#define AVOIDANCE 0
#define MIGRATION 1


/*Webots 2018b*/
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
/*Webots 2018b*/

int e_puck_matrix[2*NB_SENSORS] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS];	// Handle for the infrared distance sensors
WbDeviceTag receiver;	// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int robot_id_u;                // Unique robot ID
char* robot_name;

int fsm_state;               // leader's state of the FSM (avoidance or migration)

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
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter,out,strlen(out)+1); 
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

        reset();                    // Resetting the robot
        
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

              // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
              if ((ds_value[0] > AVOIDANCE_THRESH ||
                   ds_value[7] > AVOIDANCE_THRESH ||
                   ds_value[6] > AVOIDANCE_THRESH ||
                   ds_value[1] > AVOIDANCE_THRESH) && (fsm_state == MIGRATION)){
              
                  fsm_state = AVOIDANCE;
              }
            
            if (fsm_state == MIGRATION){ //migration
                
                /*
                 ----------------------------------------------------------------------
                 ----------------------------------------------------------------------
                 -------------------    Lavi put your code here   ---------------------
                 ----------------------------------------------------------------------
                 ----------------------------------------------------------------------
                 
                 You have to find the right msl and msr to go towards the goal
                 Replace "msl=200 and msr=200" by what your algo gives you
                 */
                
                msl=200;
                msr=200;
                
                /* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
                ----------------------------------------------------------------------
                ----------------------------------------------------------------------
                -------------------    Lavi put your code here   ---------------------
                ----------------------------------------------------------------------
                ----------------------------------------------------------------------
                */
                
            }
            else{ //avoidance state --- do only braitenberg
                
                msl = bmsl;
                msr = bmsr;
            }

            limit_and_rescale(&msl, &msr, MAX_SPEED);
            
            // Set speed
            msl_w = msl*MAX_SPEED_WEB/1000;
            msr_w = msr*MAX_SPEED_WEB/1000;

            wb_motor_set_velocity(left_motor, msl_w);
            wb_motor_set_velocity(right_motor, msr_w);
               
            //Condition to exit avoidance state
            if((fsm_state == AVOIDANCE) && (max_sens < 70)){ //Exit condition of "avoidance
                fsm_state = MIGRATION;
            }

            // Continue one step --> on change de robot!
            wb_robot_step(TIME_STEP);
         }
 }
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
 
  
  
