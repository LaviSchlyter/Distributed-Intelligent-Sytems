/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      2.0                                                         */
/* Date:         06-Oct-15                                                   */
/* Description:  Reynolds flocking with relative positions		     */
/*                                                                           */
/* Author: 	 06-Oct-15 by Ali Marjovi				     */
/* Last revision:12-Oct-15 by Florian Maushart				     */
/*****************************************************************************/



/*
  ---------------------------------- README -----------------------------------
  With this controller you can change the number of robots in the wedge formation by changing FLOCK_SIZE (2,3,4,5,6 or 7) -- change the world accordingly
  You can change the controller type proportionnal or integral
 
  TODO: You can change the formation type : column...
 */

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
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define FLOCK_SIZE	      7  // Size of flock (2,3,4,5,6 or 7) of one group for Mataric 
#define MAX_FLOCK_SIZE      7  // Maximum size of flock

#define TIME_STEP	  64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS		0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T			0.064	// Timestep (seconds)
#define DEADZONE_BEARING     8*M_PI/180 // if delta_bearing < 5 degrees : don't turn

#define ABS(x) ((x>=0)?(x):-(x))

#define LEADER_ID 0

// avoidance direction for braitenberg
#define LEFT 0
#define RIGHT 1

// ------------------------- choose your formation among 0,1,2,3 -------------------------
#define FORMATION 0
// different formations 
#define WEDGE   0
#define COLUMN  1
#define LINE    2
#define DIAMOND 3

// ------------------------- choose your controller among 0,1 -------------------------

#define CONTROLLER_TYPE 0
// different formations
#define P   0
#define PI  1

/*Webots 2018b*/
WbDeviceTag left_motor;      //handler for left wheel of the robot
WbDeviceTag right_motor;     //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];	 //Handle for the infrared distance sensors
WbDeviceTag receiver;	     //Handle for the receiver node
WbDeviceTag emitter;		 //Handle for the emitter node


int avoidance = 0;              // Boolean to avoid obstacle
int side = 0;                   // Side of avoidance (left = 0, right = 1)
int robot_id_u;           	    // Unique robot ID
char* robot_name;

// for PI controller : using tustin's approx
float integrator;
float prev_delta_bearing;

// To choose the avoidance side for Braitenberg
float x_to_leader;

float measured_range_bearing[2]; // measured range and bearing to your friend // MATARIC
float target_range[FLOCK_SIZE][FLOCK_SIZE]; // target range between the members of the swarm defining the formation 
float target_bearing[FLOCK_SIZE][FLOCK_SIZE]; // target bearing between the members of the swarm defining the formation

//enlever?
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance


// ---------------------------------------------------------------- 
// ------------------------ Formations ---------------------------- 
// ---------------------------------------------------------------- 

// ---------------- Wedge -----------------
// Only formation for which we can change  the flock  size
float wedge_range[MAX_FLOCK_SIZE][MAX_FLOCK_SIZE] =   {{   0.0    ,   0.1414  ,   0.1414  ,   0.2828  ,   0.2828  ,   0.3162  ,   0.3162  },
                                                       {  0.1414  ,    0.0    ,    0.2    ,   0.1414  ,   0.3162  ,   0.2828  ,     0.2   },
                                                       {  0.1414  ,    0.2    ,    0.0    ,   0.3162  ,   0.1414  ,    0.2    ,   0.2828  },
                                                       {  0.2828  ,   0.1414  ,   0.3162  ,    0.0    ,    0.4    ,   0.3162  ,   0.1414  },
                                                       {  0.2828  ,   0.3162  ,   0.1414  ,    0.4    ,    0.0    ,   0.1414  ,   0.3162  },
                                                       {  0.3162  ,   0.2828  ,    0.2    ,   0.3162  ,   0.1414  ,    0.0    ,     0.2   },
                                                       {  0.3162  ,    0.2    ,   0.2828  ,   0.1414  ,   0.3162  ,    0.2    ,     0.0   }};

float wedge_bearing[MAX_FLOCK_SIZE][MAX_FLOCK_SIZE] = {{   0.0    ,  5*M_PI/4 ,  3*M_PI/4 ,  5*M_PI/4 ,  3*M_PI/4 ,   2.8198  ,   3.4633  },
                                                       {  M_PI/4  ,    0.0    ,  M_PI/2   ,  5*M_PI/4 ,   1.8925  ,  3*M_PI/4 ,    M_PI   },
                                                       { 7*M_PI/4 ,  3*M_PI/2 ,    0.0    ,   4.3906  ,  3*M_PI/4 ,    M_PI   ,  5*M_PI/4 },
                                                       {  M_PI/4  ,   M_PI/4  ,   1.249   ,    0.0    ,   M_PI/2  ,   1.8925  ,  7*M_PI/4 },
                                                       { 7*M_PI/4 ,   5.0341  , 7*M_PI/4  ,  3*M_PI/2 ,    0.0    ,  5*M_PI/4 ,   4.3906  },
                                                       {  5.9614  ,  7*M_PI/4 ,    0.0    ,   5.0341  ,   M_PI/4  ,    0.0    ,  3*M_PI/2 },
                                                       {  0.3218  ,    0.0    ,  M_PI/4   ,  7*M_PI/4 ,   1.249   ,   M_PI/2  ,    0.0    }};



// -------------------------------------------------------------------------------
// ------------------------ CHANGER OU ENLEVER ----------------------------
// -------------------------------------------------------------------------
// ---------------- Column -----------------
float column_range[5][5] =  {{   0.0    ,    0.1    ,    0.1    ,    0.2    ,    0.2    },
                                               {   0.1    ,    0.0    ,    0.2    ,    0.1    ,    0.3    },
                                               {   0.1    ,    0.2    ,    0.0    ,    0.3    ,    0.1    },
                                               {   0.2    ,    0.1    ,    0.3    ,    0.0    ,    0.4    },
                                               {   0.2    ,    0.3    ,    0.1    ,    0.4    ,    0.0    }};

float column_bearing[5][5] = {{   0.0    ,  3*M_PI/2 ,   M_PI/2  ,  3*M_PI/2 ,   M_PI/2  },
                                                {  M_PI/2  ,    0.0    ,   M_PI/2  ,  3*M_PI/2 ,   M_PI/2  },
                                                { 3*M_PI/2 ,  3*M_PI/2 ,    0.0    ,  3*M_PI/2 ,   M_PI/2  },
                                                {  M_PI/2  ,   M_PI/2  ,   M_PI/2  ,    0.0    ,   M_PI/2  },
                                                { 3*M_PI/2 , 3*M_PI/2  , 3*M_PI/2  ,  3*M_PI/2 ,    0.0    }};

// ---------------- Line ----------------- CHANGER pas le bon neighbor
float line_range[5][5] =    {{   0.0    ,    0.1    ,    0.2    ,    0.3    ,    0.4    },
                                               {   0.1    ,    0.0    ,    0.1    ,    0.2    ,    0.3    },
                                               {   0.2    ,    0.1    ,    0.0    ,    0.1    ,    0.2    },
                                               {   0.3    ,    0.2    ,    0.1    ,    0.0    ,    0.1    },
                                               {   0.4    ,    0.3    ,    0.2    ,    0.1    ,    0.0    }};

float line_bearing[5][5] =  {{   0.0    ,   M_PI    ,   M_PI    ,   M_PI    ,   M_PI    },
                                               {   0.0    ,    0.0    ,   M_PI    ,   M_PI    ,   M_PI    },
                                               {   0.0    ,    0.0    ,    0.0    ,   M_PI    ,   M_PI    },
                                               {   0.0    ,    0.0    ,    0.0    ,    0.0    ,   M_PI    },
                                               {   0.0    ,    0.0    ,    0.0    ,    0.0    ,    0.0    }};

// ---------------- Diamond ----------------- CHNAGER
float diamond_range[5][5] = {{   0.0    ,   M_PI    ,   M_PI    ,   M_PI    ,   M_PI    },
                                               {   M_PI   ,    0.0    ,   M_PI    ,   M_PI    ,   M_PI    },
                                               {   0.1    ,    0.2    ,    0.0    ,   M_PI    ,   M_PI    },
                                               {   0.2    ,    0.1    ,    0.3    ,    0.0    ,   M_PI    },
                                               {   0.2    ,    0.3    ,    0.1    ,    0.4    ,    0.0    }};

float diamond_bearing[5][5] = {{   0.0    ,  5*M_PI/4 ,  3*M_PI/4 ,  5*M_PI/4 ,  3*M_PI/4 },
                                               {  M_PI/4  ,    0.0    ,  M_PI/2   ,  5*M_PI/4 ,   1.8925  },
                                               { 7*M_PI/4 ,  3*M_PI/2 ,    0.0    ,  4.3906   ,  3*M_PI/4 },
                                               {  M_PI/4  ,   M_PI/4  ,   1.249   ,    0.0    ,   M_PI/2  },
                                               { 7*M_PI/4 ,   5.0341  , 7*M_PI/4  ,  3*M_PI/2 ,    0.0    }};

// -------------------------------------------------------------------------------
// ------------------------ CHANGER OU ENLEVER ----------------------------
// -------------------------------------------------------------------------


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
	printf("robot_name: %s\n",robot_name );

	for(i=0;i<NB_SENSORS;i++){
		wb_distance_sensor_enable(ds[i],64);
       }

	wb_receiver_enable(receiver,64);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	
    printf("Reset: robot %d\n",robot_id_u);
}


/*
 * Initialiaze the matrices defining the target range and bearing depending on the wanted formation and the flock size
 */

void init_formation_matrices(void){

    int i,j;
    
    if (FORMATION == WEDGE){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = wedge_range[i][j];
                target_bearing[i][j] = wedge_bearing[i][j];
            }
        }
    }
    else if (FORMATION == COLUMN){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = column_range[i][j];
                target_bearing[i][j] = column_bearing[i][j];
            }
        }
    }
    else if (FORMATION == LINE){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = line_range[i][j];
                target_bearing[i][j] = line_bearing[i][j];
            }
        }
    }
    else if (FORMATION == DIAMOND){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = diamond_range[i][j];
                target_bearing[i][j] = diamond_bearing[i][j];
            }
        }
    }
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
 *  Each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
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
          
          // Load the last bearing to the leader - to choose the side of avoidance for braitenberg
          if (friend_robot_id == LEADER_ID){
              x_to_leader = x;
          }
		
          if ((FLOCK_SIZE >= 4) && (friend_robot_id == 1) && (robot_id_u == 3)){
        	    measured_range_bearing[0] = range;
        	    measured_range_bearing[1] = bearing;
          }
          else if ((FLOCK_SIZE >= 2) &&(friend_robot_id == 0) && (robot_id_u == 1)){
        	    measured_range_bearing[0] = range;
        	    measured_range_bearing[1] = bearing;
          }
          else if ((FLOCK_SIZE >= 3) &&(friend_robot_id == 0) && (robot_id_u == 2)){
        	    measured_range_bearing[0] = range;
        	    measured_range_bearing[1] = bearing;
          }
          else if ((FLOCK_SIZE >= 5) &&(friend_robot_id == 2) && (robot_id_u == 4)){
        	    measured_range_bearing[0] = range;
        	    measured_range_bearing[1] = bearing;
          }
          else if ((FLOCK_SIZE >= 6) &&(friend_robot_id == 4) && (robot_id_u == 5)){
                measured_range_bearing[0] = range;
                measured_range_bearing[1] = bearing;
          }
          else if ((FLOCK_SIZE >= 7) &&(friend_robot_id == 3) && (robot_id_u == 6)){
                measured_range_bearing[0] = range;
                measured_range_bearing[1] = bearing;
          }
          wb_receiver_next_packet(receiver);
    }
}

/*
 * Calculates the weight of the integral part of the controller (allowing the intagrator to 
 * discharge when the leader and the followers are not aligned 
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
    
    if ((FLOCK_SIZE >= 4) && (robot_id_u == 3)){ // friend = 1
        delta_range = measured_range_bearing[0] - target_range[3][1];
        delta_bearing = measured_range_bearing[1] - target_bearing[3][1];
    }
    else if ((FLOCK_SIZE >= 2) && (robot_id_u == 1)){ // friend = 0
        delta_range = measured_range_bearing[0] - target_range[1][0];
        delta_bearing = measured_range_bearing[1] - target_bearing[1][0];
    }
    else if ((FLOCK_SIZE >= 3) && (robot_id_u == 2)){ // friend = 0
        delta_range = measured_range_bearing[0] - target_range[2][0];
        delta_bearing = measured_range_bearing[1] - target_bearing[2][0];
    }
    else if ((FLOCK_SIZE >= 5) && (robot_id_u == 4)){ // friend = 2
        delta_range = measured_range_bearing[0] - target_range[4][2];
        delta_bearing = measured_range_bearing[1] - target_bearing[4][2];
    }
    else if ((FLOCK_SIZE >= 6) && (robot_id_u == 5)){ // friend = 4
        delta_range = measured_range_bearing[0] - target_range[5][4];
        delta_bearing = measured_range_bearing[1] - target_bearing[5][4];
    }
    else if ((FLOCK_SIZE >= 7) && (robot_id_u == 6)){ // friend = 3
        delta_range = measured_range_bearing[0] - target_range[6][3];
        delta_bearing = measured_range_bearing[1] - target_bearing[6][3];
    }
    
    if (CONTROLLER_TYPE == P){
        u = Ku*delta_range*cosf(delta_bearing);// Compute forward control
        w = Kw*delta_bearing;// Compute rotational control
    }
    else if (CONTROLLER_TYPE == PI){
        integrator = integrator + DELTA_T/2.0*(prev_delta_bearing*sigmoid(prev_delta_bearing) + delta_bearing*sigmoid(delta_bearing));
        u = Ku*delta_range*cosf(delta_bearing) + Ki*integrator;// Compute forward control
        w = Kw*delta_bearing;// Compute rotational control
        prev_delta_bearing = delta_bearing;
    }
                  	  
    // Convert to wheel speeds! msl: motor speed left
    // ???????????????????????????????????????????? PK * 1000 ????????????????????????????????????????????????????????????????????????????????????????
    *msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    limit(msl,MAX_SPEED);
    limit(msr,MAX_SPEED);
}

/*
 * A enlever
*/

void print_target(void){

   int i,j;
            
   printf("target bearing\n");
   for(i = 0; i < FLOCK_SIZE; i++){
      for(j = 0; j < FLOCK_SIZE; j++){
                    
          printf("%f, ", target_bearing[i][j]);
      }
   printf("\n");
   }
            
}
            


// the main function
int main(){
       
    int msl = 0;                    // Left wheel speed
    int msr = 0;			// Right wheel speed
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int i;                          // Loop counter
    int ds_value[NB_SENSORS];       // Array for the distance sensor readings
    int max_sens;                   // Store highest sensor value
    float priority;                 // Non-linear weight to prioritize braitenberg

    reset();			        // Resetting the robot
 	
    // Initialize the range and bearing target matrices according to the chosen formation type
    init_formation_matrices();  
 	
    print_target();
       
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
    
          /* Send and get information */
          send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
          process_received_ping_messages();
        
          // name it ?
          priority = 2;
        
          // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
          
          //if (robot_id_u == 4){printf("ds[0]: %d ---- ds[7]: %d\n", ds_value[0],ds_value[7]);}
          if ((ds_value[0] > 2100 || ds_value[7] > 2100 || ds_value[6] > 2100 || ds_value[1] > 2100) && !avoidance){
              avoidance = 1;
              printf("%d ------------------ AVOIDANCE ---------------- \n", robot_id_u);
              printf("Robot %d : x_to_leader %f \n", robot_id_u, x_to_leader);
              // Choose to follow the obstacle according to the side (left or right) that will bring the robot closer to the leader
              if (x_to_leader < 0){
                  side = LEFT; // left
              }
              else{
                  side = RIGHT; // right
              }
              printf("Robot %d : side avoid %d \n", robot_id_u, side);
          }
         
          // Normal mode -- the higher the priority, the less weight for braitenberg
          if (!avoidance){
             msl -= msl*max_sens/(priority*MAX_SENS);
             msr -= msr*max_sens/(priority*MAX_SENS);
          }
          // Avoidance mode -- follow the wall
          if (avoidance){
             if(side == RIGHT){ // avoid to the right
                if(ds_value[7] > 1300){
                   msl = 140;
                   msr = 150; // supposed to be :droit?
                }
                else{
                   //bmsl=bmsl/2;
                   //bmsr=bmsr/2;
                   msl = 100;
                   msr = 150;
                }
             }
             else{ // avoid to the left
                 if(ds_value[0] > 1200){ // close so go straight forward
                    msl = 150;
                    msr = 140;
                 }
                 else{ // far away so turn to the wall
                       //bmsl=bmsl/2;
                       //bmsr=bmsr/2;
                     msl = 150;
                     msr = 100;
                 }
              }
           }
           else{ // Normal mode
               range_bearing_to_command(&msl, &msr);
           }
           
           msl += bmsl;
           msr += bmsr;
            
           if (robot_id_u == 1){
               //printf("Robot %d : avoidance %d // msr %d // msl %d\n", robot_id_u, avoidance, msr, msl);
           }
           
                
           // Set speed
           msl_w = msl*MAX_SPEED_WEB/1000;
           msr_w = msr*MAX_SPEED_WEB/1000;
                
           wb_motor_set_velocity(left_motor, msl_w);
           wb_motor_set_velocity(right_motor, msr_w);
           
           //Condition to exit avoidance state
           if(avoidance && max_sens < 70){ //Exit condition of "avoidance
              avoidance = 0;
           }

           
           // Continue one step --> on change de robot!
           wb_robot_step(TIME_STEP);
	 }
}  
  

  

