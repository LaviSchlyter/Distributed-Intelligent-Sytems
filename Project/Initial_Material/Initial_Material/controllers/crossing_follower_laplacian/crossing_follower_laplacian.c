/*****************************************************************************/
/* File:         raynolds2.c                                                 */
/* Version:      2.0                                                         */
/* Date:         06-Oct-15                                                   */
/* Description:  Reynolds flocking with relative positions             */
/*                                                                           */
/* Author:      06-Oct-15 by Ali Marjovi                     */
/* Last revision:12-Oct-15 by Florian Maushart                     */
/*****************************************************************************/

/*
 ---------------------------------- README -----------------------------------
 1) With this controller you can change the number of edges in the graph by changing NB_EDGES (4,8 or 10) for a fixed FLOCK_SIZE of 5
    - (FLOCK_SIZE = 5, NB_EDGES = 4)
    - (FLOCK_SIZE = 5, NB_EDGES = 8)
    - (FLOCK_SIZE = 5, NB_EDGES = 10)
 2) With this controller you can change the number of robots in the wedge formation by changing FLOCK_SIZE (3,4 or 5) for a fully connected graph
    - (FLOCK_SIZE = 3, NB_EDGES = 3)
    - (FLOCK_SIZE = 4, NB_EDGES = 6)
    - (FLOCK_SIZE = 5, NB_EDGES = 10)


 TODO: You can change the formation type : You can change the controller type proportionnal or integral // not sure it works
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

#define NB_SENSORS          8      // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/

#define MAX_FLOCK_SIZE      5  // Maximum size of flock
#define MAX_NB_EDGES        10 // Maximum number of edges (fully connected)

#define TIME_STEP          64  // [ms] Length of time step

#define AXLE_LENGTH         0.052    // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS        0.00628    // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205    // Wheel radius (meters)
#define DELTA_T            0.064    // Timestep (seconds)
#define DEADZONE_BEARING     8*M_PI/180 // if delta_bearing < 5 degrees : don't turn

#define ABS(x) ((x>=0)?(x):-(x))

//boolean values
#define TRUE 1
#define FALSE 0

// avoidance direction for braitenberg
#define LEFT 0
#define RIGHT 1


// two leaders
#define LEADER1_ID 0  // leader ID of robots_ID_group1
#define LEADER2_ID 5  // leader ID of robots_ID_group2

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

// update according to the list given above (lines 14 -20)
#define FLOCK_SIZE  5  // Size of flock (3,4 or 5)
#define NB_EDGES    10  // number of edges (3,4,6,8 or 10)



/*Webots 2018b*/
WbDeviceTag left_motor;      //handler for left wheel of the robot
WbDeviceTag right_motor;     //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];     //Handle for the infrared distance sensors
WbDeviceTag receiver;         //Handle for the receiver node
WbDeviceTag emitter;         //Handle for the emitter node


int avoidance = 0;              // Boolean to avoid obstacle
int side = 0;                   // Side of avoidance (left = 0, right = 1)
int robot_id_u;                    // Unique robot ID
int corresponding_robot_ID;        // Corresponding robot ID in group 1 for robots of group 2
char* robot_name;

// for PI controller : using tustin's approx
float integrator;
float prev_delta_bearing;

// To choose the avoidance side for Braitenberg
float x_to_leader;

float target_range[FLOCK_SIZE][FLOCK_SIZE]; // target range between the members of the swarm defining the formation

float meas_range_laplacian[FLOCK_SIZE];   // measured range LAPLACIAN for robot i to every robot j : directed graph e_ij != e_ji
float meas_bearing_laplacian[FLOCK_SIZE]; // measured bearing LAPLACIAN for robot i to every robot j: directed graph b_ij != b_ji

//enlever?
int e_puck_matrix[16] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance

// ----------------------------------------------------------------
// ------------------------ LAPLACIAN ----------------------------
// ----------------------------------------------------------------

// Incidence matrix
float I[FLOCK_SIZE][NB_EDGES] = {{0.0}};
// Weight matrix
float W[NB_EDGES][NB_EDGES] = {{0.0}};
// Laplacian matrix
float L[FLOCK_SIZE][FLOCK_SIZE] = {{0.0}};
// Cardinality matrix
float Cardinality[FLOCK_SIZE][FLOCK_SIZE] = {{0.0}};

// --------------------------------------------------------------------------------------------------------------------------------
// --- For first fonctionnality of the code: fix the number of nodes (flock_size = 5) and vary the number of edges (4,8 or 10) ----
// --------------------------------------------------------------------------------------------------------------------------------

// Incidence matrix: Wedge formation:  5 nodes - 4 edges (1 neighbour directional (like Mataric))
float I_5nodes_4edges[MAX_FLOCK_SIZE][4] = {{   0.0    ,   1.0   ,   1.0  ,   0.0  },
                                            {   1.0    ,  -1.0   ,   0.0  ,   0.0  },
                                            {   0.0    ,   0.0   ,  -1.0  ,   1.0  },
                                            {  -1.0    ,   0.0   ,   0.0  ,   0.0  },
                                            {   0.0    ,   0.0   ,   0.0  ,  -1.0  }};

// Incidence matrix: Wedge formation:  5 nodes - 8 edges (2-3 neighbours directional)
float I_5nodes_8edges[MAX_FLOCK_SIZE][8] = {{   0.0    ,   1.0   ,   1.0  ,   0.0  ,   0.0    ,   0.0   ,   0.0  ,  -1.0  },
                                            {   1.0    ,  -1.0   ,   0.0  ,   0.0  ,   0.0    ,   0.0   ,  -1.0  ,   0.0  },
                                            {   0.0    ,   0.0   ,  -1.0  ,   1.0  ,   0.0    ,   1.0   ,   1.0  ,   0.0  },
                                            {  -1.0    ,   0.0   ,   0.0  ,   0.0  ,   1.0    ,  -1.0   ,   0.0  ,   0.0  },
                                            {   0.0    ,   0.0   ,   0.0  ,  -1.0  ,  -1.0    ,   0.0   ,   0.0  ,   1.0  }};
                                   
// Incidence matrix: Wedge formation:  5 nodes - 10 edges (4 neighbours directional)
float I_5nodes_10edges[MAX_FLOCK_SIZE][10] ={{   0.0    ,   1.0   ,   1.0  ,   0.0  ,   0.0    ,   0.0   ,   0.0  ,  -1.0  ,   0.0  ,  -1.0  },
                                             {   1.0    ,  -1.0   ,   0.0  ,   0.0  ,   0.0    ,   0.0   ,  -1.0  ,   0.0  ,   1.0  ,   0.0  },
                                             {   0.0    ,   0.0   ,  -1.0  ,   1.0  ,   0.0    ,   1.0   ,   1.0  ,   0.0  ,   0.0  ,   0.0  },
                                             {  -1.0    ,   0.0   ,   0.0  ,   0.0  ,   1.0    ,  -1.0   ,   0.0  ,   0.0  ,   0.0  ,   1.0  },
                                             {   0.0    ,   0.0   ,   0.0  ,  -1.0  ,  -1.0    ,   0.0   ,   0.0  ,   1.0  ,  -1.0  ,   0.0  }};

           
// Cardinality matrix - 1 neighbour
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


// ---------------------------------------------------------------------------------------------------------------------------------------------
// ----------- For second fonctionnality of the code: vary the number of nodes (FLOCK_SIZE = 3,4 or 5) for a fully connected graph -------------
// ---------------------------------------------------------------------------------------------------------------------------------------------

// Incidence matrix: Wedge formation:  3 nodes - 3 edges (fully connected)
float I_3nodes_3edges[3][3] = {{   1.0    ,   1.0   ,   0.0  },
                               {  -1.0    ,   0.0   ,  -1.0  },
                               {   0.0    ,  -1.0   ,   1.0  }};

// Incidence matrix: Wedge formation:  4 nodes - 6 edges (fully connected)
float I_4nodes_6edges[4][6] = {{   0.0    ,   1.0   ,   1.0  ,   0.0  ,   0.0    ,  -1.0   },
                                {   1.0    ,  -1.0   ,   0.0  ,   0.0  ,  -1.0    ,   0.0   },
                                {   0.0    ,   0.0   ,  -1.0  ,   1.0  ,   1.0    ,   0.0   },
                                {  -1.0    ,   0.0   ,   0.0  ,  -1.0  ,   0.0    ,   1.0   }};


// --------------------------------------------------------------------------------------------------------------------------------
// --------------------------------------------------------- Formations -----------------------------------------------------------
// --------------------------------------------------------------------------------------------------------------------------------

// ---------------- Wedge -----------------
// Only formation for which we can change  the flock  size
float wedge_range[MAX_FLOCK_SIZE][MAX_FLOCK_SIZE] =   {{   0.0    ,   0.1414  ,   0.1414  ,   0.2828  ,   0.2828  },
                                                       {  0.1414  ,    0.0    ,    0.2    ,   0.1414  ,   0.3162  },
                                                       {  0.1414  ,    0.2    ,    0.0    ,   0.3162  ,   0.1414  },
                                                       {  0.2828  ,   0.1414  ,   0.3162  ,    0.0    ,    0.4    },
                                                       {  0.2828  ,   0.3162  ,   0.1414  ,    0.4    ,    0.0    }};


// CHANGER OU ENLEVER
// ---------------- Column -----------------
float column_range[5][5] =  {{   0.0    ,    0.1    ,    0.1    ,    0.2    ,    0.2    },
                             {   0.1    ,    0.0    ,    0.2    ,    0.1    ,    0.3    },
                             {   0.1    ,    0.2    ,    0.0    ,    0.3    ,    0.1    },
                             {   0.2    ,    0.1    ,    0.3    ,    0.0    ,    0.4    },
                             {   0.2    ,    0.3    ,    0.1    ,    0.4    ,    0.0    }};

// ---------------- Line ----------------- CHANGER pas le bon neighbor
float line_range[5][5] =    {{   0.0    ,    0.1    ,    0.2    ,    0.3    ,    0.4    },
                             {   0.1    ,    0.0    ,    0.1    ,    0.2    ,    0.3    },
                             {   0.2    ,    0.1    ,    0.0    ,    0.1    ,    0.2    },
                             {   0.3    ,    0.2    ,    0.1    ,    0.0    ,    0.1    },
                             {   0.4    ,    0.3    ,    0.2    ,    0.1    ,    0.0    }};

// ---------------- Diamond ----------------- CHNAGER
float diamond_range[5][5] = {{   0.0    ,   M_PI    ,   M_PI    ,   M_PI    ,   M_PI    },
                             {   M_PI   ,    0.0    ,   M_PI    ,   M_PI    ,   M_PI    },
                             {   0.1    ,    0.2    ,    0.0    ,   M_PI    ,   M_PI    },
                             {   0.2    ,    0.1    ,    0.3    ,    0.0    ,   M_PI    },
                             {   0.2    ,    0.3    ,    0.1    ,    0.4    ,    0.0    }};


// ------------------------ Crossing groups  ----------------------------

// when reducing the size of the flock, first we remove robot_ID 3, then 4
int robots_ID_group1[MAX_FLOCK_SIZE]= {0,2,1,4,3};


// when reducing the size of the flock, first we remove robot_ID 9, then 8
int robots_ID_group2[MAX_FLOCK_SIZE]= {5,6,7,8,9};


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
 * Initialiaze the matrices (Incidence, weight and cardinality) needed when using the laplacian controller
 */
 
void init_laplacian_matrices(void){
    
    int i,j,k;
    
    // First functionallity of this code: fix the number of nodes (flock_size = 5) and vary the number of edges (4,8 or 10)
    if (MAX_FLOCK_SIZE == FLOCK_SIZE){
        //printf("First functionality \n");
          if (NB_EDGES == 4){  // 1 neighbour
              for(i=0; i<MAX_FLOCK_SIZE; i++) {
                  for(j=0; j<NB_EDGES; j++) {
                      I[i][j] = I_5nodes_4edges[i][j];
                  }
                      
                  for(k=0; k<MAX_FLOCK_SIZE; k++) {
                      Cardinality[i][k] = C_1neighbour[i][k];
                  }
               }
           }
    
          if (NB_EDGES == 8){ // 2-3 neighbours
              for(i=0; i<MAX_FLOCK_SIZE; i++) {
                  for(j=0; j<NB_EDGES; j++) {
                      I[i][j] = I_5nodes_8edges[i][j];
                  }
                  
                  for(k=0; k<MAX_FLOCK_SIZE; k++) {
                      Cardinality[i][k] = C_2neighbour[i][k];
                  }
              }
          }
        
          if (NB_EDGES == 10){  // 4 neighbours
              for(i=0; i<MAX_FLOCK_SIZE; i++) {
                  for(j=0; j<NB_EDGES; j++) {
                      I[i][j] = I_5nodes_10edges[i][j];
                  }
                  
                  for(k=0; k<MAX_FLOCK_SIZE; k++) {
                      Cardinality[i][k] = C_4neighbour[i][k];
                  }
              }
          }
    }
    else{ // Second functionallity of this code: vary the number of nodes (3,4 or 5) for a fully connected graph (the case 5 is handeld above)
        
        //printf("Second functionality \n");
        
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
 * Initialiaze the matrices defining the target range and bearing depending on the wanted formation
 */

void init_formation_matrices(void){

    int i,j;
    
    if (FORMATION == WEDGE){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = wedge_range[i][j];
            }
        }
    }
    else if (FORMATION == COLUMN){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = column_range[i][j];
            }
        }
    }
    else if (FORMATION == LINE){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = line_range[i][j];
            }
        }
    }
    else if (FORMATION == DIAMOND){
        for(i=0; i<FLOCK_SIZE; i++) {
            for(j=0; j<FLOCK_SIZE; j++) {
                target_range[i][j] = diamond_range[i][j];
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
 * The two groups behave the same, hence each robot of group 2 behaves like one robot of group 1.
 * This function tells to the robots of group 2 their corresponding robot ID in group 1
 */
void corresponding_robot_ID_in_group1(void){

    if (robot_id_u == 5){
        corresponding_robot_ID = 0;
    }
    else if (robot_id_u == 7){
        corresponding_robot_ID = 1;
    }
    else if (robot_id_u == 6){
        corresponding_robot_ID = 2;
    }
    else if (robot_id_u == 9){
        corresponding_robot_ID = 3;
    }
    else if (robot_id_u == 8){
        corresponding_robot_ID = 4;
    }
}


/*
 * Check if the robot_ID is in group 1 (return True if that's the case)
 */
bool robotID_in_group1(void){

   int i;
   bool is_in_group1 = FALSE;
   
   for(i=0; i<FLOCK_SIZE; i++){
   
      if (robot_id_u==robots_ID_group1[i]){
         is_in_group1 = TRUE;
      }
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
          printf("friend_robot_id %d\n", friend_robot_id);
        
          // Load the last bearing to the leader - to choose the side of avoidance for braitenberg
          if ((robotID_in_group1() && (friend_robot_id == LEADER1_ID)) ||
              (!robotID_in_group1() && (friend_robot_id == LEADER2_ID))){
              x_to_leader = x;
          }
        
          // group 1
          if (robotID_in_group1()){
               meas_range_laplacian[friend_robot_id] = range;
               meas_bearing_laplacian[friend_robot_id] = bearing;
          }
          else {
          // group 2
              meas_range_laplacian[corresponding_robot_ID] = range;
              meas_bearing_laplacian[corresponding_robot_ID] = bearing;
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
         
         index = corresponding_robot_ID;
         printf(" Robot %d , I'm in group 2, my index is %d \n", robot_id_u,index);
    }
    
           
    for (i = 0; i < FLOCK_SIZE; i++){
        
         card_nb += Cardinality[index][i];
         // previous version
         //e_xi += - (meas_range_laplacian[i] - target_range[robot_id_u][i])*L[robot_id_u][i]*cosf(meas_bearing_laplacian[i] - target_bearing[robot_id_u][i]);
         //e_yi += - (meas_range_laplacian[i] - target_range[robot_id_u][i])*L[robot_id_u][i]*ABS(sinf(meas_bearing_laplacian[i] - target_bearing[robot_id_u][i]));
            
         if (Cardinality[index][i] == 1){
             e_xi += - (meas_range_laplacian[i] - target_range[index][i])*L[index][i]*cosf(meas_bearing_laplacian[i]);
             e_yi += - (meas_range_laplacian[i] - target_range[index][i])*L[index][i]*sinf(meas_bearing_laplacian[i]);
         }
    }
       
    delta_range = sqrt(pow(e_xi,2) + pow(e_yi,2));
    delta_bearing = atan2f(e_yi, e_xi);
          
    // proportional controller - Falconi_Riccardo_Tesi
    if (CONTROLLER_TYPE == P){
        u = Ku*delta_range;// Compute forward control
        w = Kw*delta_bearing;// Compute rotational control
    }
    
    // proportional + integral controller - ---------------------enlever?------------------------
    else if (CONTROLLER_TYPE == PI){
        integrator = integrator + DELTA_T/2.0*(prev_delta_bearing*sigmoid(prev_delta_bearing) + delta_bearing*sigmoid(delta_bearing));
        //u = Ku*delta_range*cosf(delta_bearing) + Ki*integrator;// Compute forward control
        u = Ku*delta_range+ Ki*integrator;// Compute forward control
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

/*
 * ENLEVER
 *
*/

void print_laplacian(void){

  int i,j;
  
  printf("Incidence\n");
  for(i = 0; i < FLOCK_SIZE; i++){
      for(j = 0; j < NB_EDGES; j++){
          
          printf("%f, ", I[i][j]);
      }
      printf("\n");
  }
  
  printf("Weight\n");
  for(i = 0; i < NB_EDGES; i++){
      for(j = 0; j < NB_EDGES; j++){
          
          printf("%f, ", W[i][j]);
      }
      printf("\n");
  }

  printf("Laplacian\n");
  for(i = 0; i < FLOCK_SIZE; i++){
      for(j = 0; j < FLOCK_SIZE; j++){
          
          printf("%f, ", L[i][j]);
      }
      printf("\n");
  }
  
  printf("Cardinality\n");
  for(i = 0; i < FLOCK_SIZE; i++){
      for(j = 0; j < FLOCK_SIZE; j++){
          
          printf("%f, ", Cardinality[i][j]);
      }
      printf("\n");
  }
  
   printf("target range \n");
  for(i = 0; i < FLOCK_SIZE; i++){
      for(j = 0; j < FLOCK_SIZE; j++){
          
          printf("%f, ", target_range[i][j]);
      }
      printf("\n");
  }
  
}
                 
                
            


// the main function
int main(){
       
    int msl = 0;                    // Left wheel speed
    int msr = 0;            // Right wheel speed
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int i;                          // Loop counter
    int ds_value[NB_SENSORS];       // Array for the distance sensor readings
    int max_sens;                   // Store highest sensor value
    float priority;                 // Non-linear weight to prioritize braitenberg

    reset();                    // Resetting the robot
     
    // Initilaize the corresponding robot ID in group 1 if the robot is in group 2
    if (!robotID_in_group1()){
        corresponding_robot_ID_in_group1();
        printf("Robot %d: I'm in group 2 // corresp = %d \n", robot_id_u, corresponding_robot_ID);
    }
    else{
        printf("Robot %d: I'm in group 1 // corresp = %d \n", robot_id_u, corresponding_robot_ID);
    }
    
    // Initialize the range and bearing target matrices according to the chosen formation type
    init_formation_matrices();
     
    // Initialize the matrices (C,I,W)
    init_laplacian_matrices();
    
    // Compute
    compute_laplacian();
    
    //print_laplacian();
       
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
  

  


