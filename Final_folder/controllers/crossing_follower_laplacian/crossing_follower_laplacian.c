/*****************************************************************************************************************************/
/* File:         crossing_follower_laplacian.c                                 */
/* Version:      1.0                                                           */
/* Date:         06-Jun-21                                                     */
/* Description:  Formation with relative positions in a world where two        */
/*               teams of robots are crossing: follower controller (laplacian) */
/*                                                                             */
/* Author:       Tifanny Portela                                               */
/*****************************************************************************************************************************/


/* Tunable parameters:
 
 1) With this controller you can change the number of edges in the graph by changing NB_EDGES (4,8 or 10) for a fixed FLOCK_SIZE of 5. Here are the possible combinations (change the values accordingly at line 35-36):
    - (FLOCK_SIZE = 5, NB_EDGES = 4)
    - (FLOCK_SIZE = 5, NB_EDGES = 8)
    - (FLOCK_SIZE = 5, NB_EDGES = 10)
 2) With this controller you can change the number of robots in the formation by changing FLOCK_SIZE (3,4 or 5) for a fully connected graph. Here are the possible combinations (change the values accordingly at line 35-36):
    - (FLOCK_SIZE = 3, NB_EDGES = 3)
    - (FLOCK_SIZE = 4, NB_EDGES = 6)
    - (FLOCK_SIZE = 5, NB_EDGES = 10) s*/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

// ------------------------- Choose the flock size and the number of edges of the graph -------------------------
#define FLOCK_SIZE  5  // Size of flock (3,4 or 5)
#define NB_EDGES    8  // number of edges (3,4,6,8 or 10)

// ------------------------- Choose your world  -------------------------
#define CROSSING 0
#define TEST_CROSSING 1
#define WORLD TEST_CROSSING

#define NB_SENSORS          8      // Number of distance sensors
#define MIN_SENS          350      // Minimum sensibility value
#define MAX_SPEED         800      // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots

#define AVOIDANCE_THRESH    1800  // Threshold above which the robot enters obstacle avoidance state
#define MAX_FLOCK_SIZE      5     // Maximum size of flock
#define MAX_NB_EDGES        10    // Maximum number of edges (fully connected)
#define TIME_STEP           64    // Length of time step in [ms]

#define AXLE_LENGTH         0.052    // Distance between wheels of robot (meters)
#define WHEEL_RADIUS        0.0205   // Wheel radius (meters)
#define DELTA_T             0.064    // Timestep (seconds)

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
WbDeviceTag ds[NB_SENSORS];  //handler for the infrared distance sensors
WbDeviceTag receiver;        //handler for the receiver node
WbDeviceTag emitter;         //handler for the emitter node


int fsm_state;                     // finite state machine's state
int robot_id_u;                    // unique robot ID
int corresponding_robot_ID;        // Corresponding robot ID in group 1 for robots of group 2
char* robot_name;

float meas_range_laplacian[MAX_FLOCK_SIZE];   // measured range for robot i to every robot j
float meas_bearing_laplacian[MAX_FLOCK_SIZE]; // measured bearing for robot i to every robot j

// Incidence matrix
float I[FLOCK_SIZE][NB_EDGES] = {{0.0}};
// Weight matrix
float W[NB_EDGES][NB_EDGES] = {{0.0}};
// Laplacian matrix
float L[FLOCK_SIZE][FLOCK_SIZE] = {{0.0}};
// Cardinality matrix
float Cardinality[FLOCK_SIZE][FLOCK_SIZE] = {{0.0}};


// Incidence matrix: 3 nodes - 3 edges (fully connected)
float I_3nodes_3edges[3][3] = {{   1.0    ,   1.0   ,   0.0  },
                               {  -1.0    ,   0.0   ,  -1.0  },
                               {   0.0    ,  -1.0   ,   1.0  }};

// Incidence matrix: 4 nodes - 6 edges (fully connected)
float I_4nodes_6edges[4][6] = {{   0.0    ,   1.0   ,   1.0  ,   0.0  ,   0.0    ,  -1.0   },
                               {   1.0    ,  -1.0   ,   0.0  ,   0.0  ,  -1.0    ,   0.0   },
                               {   0.0    ,   0.0   ,  -1.0  ,   1.0  ,   1.0    ,   0.0   },
                               {  -1.0    ,   0.0   ,   0.0  ,  -1.0  ,   0.0    ,   1.0   }};
   
// Incidence matrix: 5 nodes - 10 edges (fully connected)
float I_5nodes_10edges[MAX_FLOCK_SIZE][10] ={{   0.0    ,   1.0   ,   1.0  ,   0.0  ,   0.0    ,   0.0   ,   0.0  ,   1.0  ,   0.0  ,   1.0  },
                                             {   1.0    ,  -1.0   ,   0.0  ,   0.0  ,   0.0    ,   0.0   ,  -1.0  ,   0.0  ,   1.0  ,   0.0  },
                                             {   0.0    ,   0.0   ,  -1.0  ,   1.0  ,   0.0    ,   1.0   ,   1.0  ,   0.0  ,   0.0  ,   0.0  },
                                             {  -1.0    ,   0.0   ,   0.0  ,   0.0  ,   1.0    ,  -1.0   ,   0.0  ,   0.0  ,   0.0  ,  -1.0  },
                                             {   0.0    ,   0.0   ,   0.0  ,  -1.0  ,  -1.0    ,   0.0   ,   0.0  ,  -1.0  ,  -1.0  ,   0.0  }};

// Cardinality matrix - 1-2 neighbours
int C_1neighbour[MAX_FLOCK_SIZE][MAX_FLOCK_SIZE] = {{   0    ,   1   ,   1  ,   0  ,   0   },
                                                    {   1    ,   0   ,   0  ,   1  ,   0   },
                                                    {   1    ,   0   ,   0  ,   0  ,   1   },
                                                    {   0    ,   1   ,   0  ,   0  ,   0   },
                                                    {   0    ,   0   ,   1  ,   0  ,   0   }};

// Cardinality matrix - 2-3 neighbours
int C_2neighbour[MAX_FLOCK_SIZE][MAX_FLOCK_SIZE] = {{   0    ,   1   ,   1  ,   0  ,   1   },
                                                    {   1    ,   0   ,   1  ,   1  ,   0   },
                                                    {   1    ,   1   ,   0  ,   1  ,   1   },
                                                    {   0    ,   1   ,   1  ,   0  ,   1   },
                                                    {   1    ,   0   ,   1  ,   1  ,   0   }};
                                     
// Cardinality matrix - 4 neighbours
int C_4neighbour[MAX_FLOCK_SIZE][MAX_FLOCK_SIZE] = {{   0    ,   1   ,   1  ,   1  ,   1   },
                                                    {   1    ,   0   ,   1  ,   1  ,   1   },
                                                    {   1    ,   1   ,   0  ,   1  ,   1   },
                                                    {   1    ,   1   ,   1  ,   0  ,   1   },
                                                    {   1    ,   1   ,   1  ,   1  ,   0   }};

// target ranges between flock memebers
float target_range[MAX_FLOCK_SIZE][MAX_FLOCK_SIZE] =  {{   0.0    ,   0.1414  ,   0.1414  ,   0.2828  ,   0.2828  },
                                                       {  0.1414  ,    0.0    ,    0.2    ,   0.1414  ,   0.3162  },
                                                       {  0.1414  ,    0.2    ,    0.0    ,   0.3162  ,   0.1414  },
                                                       {  0.2828  ,   0.1414  ,   0.3162  ,    0.0    ,    0.4    },
                                                       {  0.2828  ,   0.3162  ,   0.1414  ,    0.4    ,    0.0    }};

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
        ds[i]=wb_robot_get_device(s);    // the device name is specified in the world file
        s[2]++;                // increases the device number
    }
    robot_name=(char*) wb_robot_get_name();

    for(i=0;i<NB_SENSORS;i++){
        wb_distance_sensor_enable(ds[i],64);
       }

    wb_receiver_enable(receiver,64);

    //Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
    sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
}



/*
 * Initialiaze the matrices (Incidence, weight and cardinality) needed when using the laplacian controller
 */
 
void init_laplacian_matrices(void){
    
    int i,j,k;
    
    // First functionallity of this code: fix the number of nodes (flock_size = 5) and vary the number of edges (4,8 or 10)
    if (MAX_FLOCK_SIZE == FLOCK_SIZE){
        for(i=0; i<MAX_FLOCK_SIZE; i++) {
            for(j=0; j<NB_EDGES; j++) {
                I[i][j] = I_5nodes_10edges[i][j];
            }
            for(k=0; k<MAX_FLOCK_SIZE; k++) {
                 if (NB_EDGES == 4){ // 1-2 neighbours
                     Cardinality[i][k] = C_1neighbour[i][k];
                 }
                 else if (NB_EDGES == 8){ // 2-3 neighbours
                     Cardinality[i][k] = C_2neighbour[i][k];
                 }
                 else if (NB_EDGES == 10){ // 4 neighbours
                     Cardinality[i][k] = C_4neighbour[i][k];
                 }
                 else{
                     printf("This combination (FLOCK_SIZE = %d, NB_EDGES = %d) is not possible\n",FLOCK_SIZE, NB_EDGES);
                 }
            }
        }
    }
    else{ // Second functionallity of this code: vary the number of nodes (3,4 or 5) for a fully connected graph (the case 5 is handeld above)
        for(i=0; i<FLOCK_SIZE; i++) {
            for(k=0; k<FLOCK_SIZE; k++) {
                Cardinality[i][k] = C_4neighbour[i][k];
            }
            if (FLOCK_SIZE == 3){
               for(j=0; j<3; j++) {
                    I[i][j] = I_3nodes_3edges[i][j];
               }
            }
            if (FLOCK_SIZE == 4){
               for(j=0; j<6; j++) {
                    I[i][j] = I_4nodes_6edges[i][j];
               }
            }
        }
    }
   
    // Init weight matrix - identity matrix
    for(i=0;i<NB_EDGES;i++){
        for(j=0;j<NB_EDGES;j++){
            if (i==j){
                W[i][j] = 1.0;
            }
        }
    }
}


/*
 * The two groups behave the same, hence each robot of group 2 behaves like one robot of group 1.
 * This function tells to the robots of group 2 their corresponding robot ID in group 1 for the crossing world 
 */
int corresponding_robot_ID_in_group1(int robot_id){

    int corresponding_robot_ID = 0;

    if (robot_id == 5){
        corresponding_robot_ID = 0;
    }
    else if (robot_id == 7){
        corresponding_robot_ID = 1;
    }
    else if (robot_id == 6){
        corresponding_robot_ID = 2;
    }
    else if (robot_id == 9){
        corresponding_robot_ID = 3;
    }
    else if (robot_id == 8){
        corresponding_robot_ID = 4;
    }
    return corresponding_robot_ID;  
}

/*
 * The two groups behave the same, hence each robot of group 2 behaves like one robot of group 1.
 * This function tells to the robots of group 2 their corresponding robot ID in group 1 for the test crossing world 
 */
int corresponding_robot_ID_in_group1t(int robot_id){

    int corresponding_robot_ID = 0;

    if (robot_id == 5){
        corresponding_robot_ID = 0;
    }
    else if (robot_id == 7){
        corresponding_robot_ID = 2;
    }
    else if (robot_id == 6){
        corresponding_robot_ID = 1;
    }
    else if (robot_id == 9){
        corresponding_robot_ID = 4;
    }
    else if (robot_id == 8){
        corresponding_robot_ID = 3;
    }
    return corresponding_robot_ID;  
}


/*
 * Check if the robot number i is in one of the groups (return True if that's the case)
 */
 
bool i_is_in_the_list_for_reduced_flock_size(int index){
    
    bool in_the_list = TRUE;
    if (index >= FLOCK_SIZE){
    
       in_the_list = FALSE;
    }
    return in_the_list;
}


/*
 * Check if the robot_ID is in group 1 (return True if that's the case)
 */
bool robotID_in_group1(void){

   bool is_in_group1 = FALSE;
   if (robot_id_u < FLOCK_SIZE){
     is_in_group1 = TRUE;
   }
   return is_in_group1;
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
    char *inbuffer;    // Buffer for the receiver node
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
        
          // in group 1 and it's one of my friends (ID 0 to 4)
          if ((robotID_in_group1()) && (friend_robot_id <= 4)) {
               meas_range_laplacian[friend_robot_id] = range;
               meas_bearing_laplacian[friend_robot_id] = bearing;
               
          }// in group 2 and it's one of my friends (ID 5 to 9)
          else if ((!robotID_in_group1()) && (friend_robot_id > 4)){ 
          
              if (WORLD == TEST_CROSSING){
                 meas_range_laplacian[corresponding_robot_ID_in_group1t(friend_robot_id)] = range;
                 meas_bearing_laplacian[corresponding_robot_ID_in_group1t(friend_robot_id)] = bearing;
              }
              else{
                 meas_range_laplacian[corresponding_robot_ID_in_group1(friend_robot_id)] = range;
                 meas_bearing_laplacian[corresponding_robot_ID_in_group1(friend_robot_id)] = bearing;
              }
        
              
          }
        
          wb_receiver_next_packet(receiver);
    }
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
 * Given the measured range and bearing of your friend, determine the control commands
 * (u = linear velocity, omega = angular velocity) to match your target range and bearing
*/

void range_bearing_to_command(int *msl, int *msr){

    float Ku = 0.2;  // Forward control coefficient 0.2
    float Kw = 0.4;  // Rotational control coefficient 0.5
    float delta_bearing = 0.0;
    float delta_range = 0.0;
    float u,w;
    int i;
    float e_xi = 0;
    float e_yi = 0;
    int card_nb = 0;
    int index = 0;
    
    // group 1
    if (robotID_in_group1()){
        index = robot_id_u;
    }
    else{
    // group 2
    
        if (WORLD == TEST_CROSSING){
           index = corresponding_robot_ID_in_group1t(robot_id_u);
        }
        else{
           index = corresponding_robot_ID_in_group1(robot_id_u);
        }
    }
          
    for (i = 0; i < MAX_FLOCK_SIZE; i++){
        if (i_is_in_the_list_for_reduced_flock_size(i)){
        
             card_nb += Cardinality[index][i];
             if (Cardinality[index][i] == 1){
                 e_xi += - (meas_range_laplacian[i] - target_range[index][i])*L[index][i]*cosf(meas_bearing_laplacian[i]);
                 e_yi += - (meas_range_laplacian[i] - target_range[index][i])*L[index][i]*sinf(meas_bearing_laplacian[i]);
             }
        }
    }
       
    delta_range = sqrt(pow(e_xi,2) + pow(e_yi,2));
    delta_bearing = atan2f(e_yi, e_xi);
          
    // Proportional controller
    u = Ku*delta_range;
    w = Kw*delta_bearing;
                  
    *msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
    limit_and_rescale(msl, msr, MAX_SPEED);
}

/*
 * Compute the laplacian matrix given the weight and the incidence matrices: L = IWI'
*/

void compute_laplacian(void){

  int i,j,k;
  float matrix_I_times_W[FLOCK_SIZE][NB_EDGES] = {{0.0}};
  
  for(i = 0; i < FLOCK_SIZE; i++){
      for(j = 0; j < NB_EDGES; j++){
          for(k = 0; k < NB_EDGES; k++){
              
              matrix_I_times_W[i][j] += I[i][k]*W[k][j];
          }
      }
  }
 
 
  for(i = 0; i < FLOCK_SIZE; i++){
      for(j = 0; j < FLOCK_SIZE; j++){
          for(k = 0; k < NB_EDGES; k++){
              
              L[i][j] += matrix_I_times_W[i][k]*I[j][k];
          }
      }
  }

}
                 

// the main function
int main(){
       
    int msl = 0;                    // Left wheel speed
    int msr = 0;                    // Right wheel speed
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int i;                          // Loop counter
    int ds_value[NB_SENSORS];       // Array for the distance sensor readings
    int max_sens;                   // Store highest sensor value

    reset();                        // Resetting the robot
    init_laplacian_matrices();      // Initialize the matrices (C,I,W)
    compute_laplacian();            // Compute laplacian

    fsm_state = FORMATION;          // initial FSM's state: formation
       
    for(;;){// Forever
        
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
        
         // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
         if ((ds_value[0] > AVOIDANCE_THRESH ||
              ds_value[7] > AVOIDANCE_THRESH ||
              ds_value[6] > AVOIDANCE_THRESH ||
              ds_value[1] > AVOIDANCE_THRESH) && (fsm_state == FORMATION)){
            fsm_state = AVOIDANCE;
         }
         if (fsm_state == FORMATION){ //formation state --- do only formation
            range_bearing_to_command(&msl, &msr);//formation command
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
        if((fsm_state == AVOIDANCE) && (max_sens < FORMATION_THRESH)){ //Exit condition of "avoidance
           fsm_state = FORMATION;
        }

        // Continue one step
        wb_robot_step(TIME_STEP);
    }
}
