/*****************************************************************************************************************************/
/* File:         pso_simplified_follower_formation.c                           */
/* Version:      1.0                                                           */
/* Date:         01-Mai-21                                                     */
/* Description:  PSO for the formation (laplacian) with a group                */
/*               public  homogeneous configuration for the followers.          */
/*                                                                             */
/* Author:       Paco Mermoud                                                  */
/*****************************************************************************************************************************/

/* Tunable parameters:
 1) With this controller you can change the number of edges in the graph by changing NB_EDGES (4,8 or 10) for a fixed FLOCK_SIZE of 5. Here are the possible combinations (change the values accordingly at line 35-36):
    - (FLOCK_SIZE = 5, NB_EDGES = 4)
    - (FLOCK_SIZE = 5, NB_EDGES = 8)
    - (FLOCK_SIZE = 5, NB_EDGES = 10)
 2) With this controller you can change the number of robots in the formation by changing FLOCK_SIZE (3,4 or 5) for a fully connected graph. Here are the possible combinations (change the values accordingly at line 35-36):
    - (FLOCK_SIZE = 3, NB_EDGES = 3)
    - (FLOCK_SIZE = 4, NB_EDGES = 6)
    - (FLOCK_SIZE = 5, NB_EDGES = 10) */

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

#define NB_SENSORS          8      // Number of distance sensors
#define MIN_SENS          350      // Minimum sensibility value
#define MAX_SPEED         800      // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed Webots

#define MAX_FLOCK_SIZE      5     // Maximum size of flock
#define MAX_NB_EDGES        10    // Maximum number of edges (fully connected)
#define TIME_STEP           64    // Length of time step in [ms]

#define AXLE_LENGTH         0.052    // Distance between wheels of robot (meters)
#define WHEEL_RADIUS        0.0205   // Wheel radius (meters)
#define DELTA_T             0.064    // Time step (seconds)

// PSO -------------------- Choose the number of Sim step -----------------------------------------------------------------
#define SIM_STEPS 1200                // Number of simulation steps/iterations
#define DATASIZE 4                    // Number of elements in particle
#define SCALING_P 100                 // Scaling factor for lower parameters in PSO (Put every parameters at the same range of magnitude)

//States of FSM
#define AVOIDANCE 0
#define FORMATION 1

WbDeviceTag left_motor;      //handler for left wheel of the robot
WbDeviceTag right_motor;     //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS];	 //handler for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node for flocking
WbDeviceTag emitter;		// Handle for the emitter node for flocking
WbDeviceTag rec_pso;		// Handle for the receiver node for PSO
WbDeviceTag emit_pso;		// Handle for the emitter node for PSO

int fsm_state = 0;           // Finite state machine's state
int robot_id_u;	             // Unique robot ID
char* robot_name;
int robot_verbose=0;         // boolean to know for which robot we print


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

// Matrice for BraitenBerg avoidance
int e_puck_matrix[2*NB_SENSORS] = {17,29,34,10,8,-60,-64,-84,
                        -80,-66,-62,8,10,36,28,18}; // for obstacle avoidance


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
        ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
        s[2]++;			// increases the device number
    }
    robot_name=(char*) wb_robot_get_name();
    printf("robot_name: %s\n",robot_name );

    for(i=0;i<NB_SENSORS;i++){
        wb_distance_sensor_enable(ds[i],TIME_STEP);
    }

    wb_receiver_enable(receiver,TIME_STEP/2);

    //Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
    sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name

    if(robot_id_u==1){ // print robot data
        robot_verbose=1;
    }

    // Receiver and emitter PSO
    emit_pso = wb_robot_get_device("emitter_epuck_pso");
    rec_pso = wb_robot_get_device("receiver_epuck_pso");
    wb_receiver_enable(rec_pso, TIME_STEP/2);

}


/*
 * Initialize the matrices (Incidence, weight and cardinality) needed when using the laplacian controller
 */
void init_laplacian_matrices(void){

    int i,j,k;   //iterators for loops

    // First functionality of this code: fix the number of nodes (flock_size = 5) and vary the number of edges (4,8 or 10)
    if (MAX_FLOCK_SIZE == FLOCK_SIZE){
        for(i=0; i<MAX_FLOCK_SIZE; i++) {
            for(j=0; j<NB_EDGES; j++) {
                I[i][j] = I_5nodes_10edges[i][j];
            }
            for(k=0; k<MAX_FLOCK_SIZE; k++) {
                 if (NB_EDGES == 4){ // 1-2 neighbors
                     Cardinality[i][k] = C_1neighbour[i][k];
                 }
                 else if (NB_EDGES == 8){ // 2-3 neighbors
                     Cardinality[i][k] = C_2neighbour[i][k];
                 }
                 else if (NB_EDGES == 10){ // 4 neighbors
                     Cardinality[i][k] = C_4neighbour[i][k];
                 }
                 else{
                     printf("This combination (FLOCK_SIZE = %d, NB_EDGES = %d) is not possible\n",FLOCK_SIZE, NB_EDGES);
                 }
            }
        }
    }
    else{ // Second functionality of this code: vary the number of nodes (3,4 or 5) for a fully connected graph (the case 5 is handeld above)
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

          meas_range_laplacian[friend_robot_id] = range;
          meas_bearing_laplacian[friend_robot_id] = bearing;

          wb_receiver_next_packet(receiver);
    }
}


/*
 * Calculates the weight of the integral part of the controller (allowing the integrator to
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

void range_bearing_to_command(int *msl, int *msr, float Ku, float Kw){
    float delta_bearing = 0.0;
    float delta_range = 0.0;
    float u,w;
    int i;
    float e_xi = 0;
    float e_yi = 0;
    int card_nb = 0;

    for (i = 0; i < FLOCK_SIZE; i++){

         card_nb += Cardinality[robot_id_u][i];

         if (Cardinality[robot_id_u][i] == 1){
             e_xi += - (meas_range_laplacian[i] - target_range[robot_id_u][i])*L[robot_id_u][i]*cosf(meas_bearing_laplacian[i]);
             e_yi += - (meas_range_laplacian[i] - target_range[robot_id_u][i])*L[robot_id_u][i]*sinf(meas_bearing_laplacian[i]);
         }
    }

    e_xi = e_xi/(card_nb + 1);
    e_yi = e_yi/(card_nb + 1);

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

/*
 * Compute the simulation for SIM_STEPS iteration
*/
void simulation_webot(double weights[DATASIZE+1]){

    int msl = 0;                    // Left wheel speed
    int msr = 0;                    // Right wheel speed
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int i,j;                          // Loop counter
    int ds_value[NB_SENSORS];       // Array for the distance sensor readings
    int max_sens;                   // Store highest sensor value
    init_laplacian_matrices();      // Initialize the matrices (C,I,W)
    compute_laplacian();            // Compute laplacian
    fsm_state = FORMATION;           // initial FSM's state: formation

    // Simulation
    for(j=0;j<SIM_STEPS;j++){
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
            bmsl += e_puck_matrix[NB_SENSORS+i] * ds_value[i];
         }

          // Adapt Braitenberg values (empirical tests)
          bmsl/=MIN_SENS; bmsr/=MIN_SENS;
          bmsl+=66; bmsr+=72;

          /* Send and get information */
          send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
          process_received_ping_messages();

         // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
         if ((ds_value[0] > weights[0] ||
              ds_value[7] > weights[0] ||
              ds_value[6] > weights[0] ||
              ds_value[1] > weights[0]) && (fsm_state == FORMATION)){
              fsm_state = AVOIDANCE;
         }
         if (fsm_state == FORMATION){ //formation state --- do only formation
            range_bearing_to_command(&msl, &msr, weights[2]/SCALING_P, weights[3]/SCALING_P);//formation command
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
         if((fsm_state == AVOIDANCE) && (max_sens < weights[1])){ //Exit condition of "avoidance
           fsm_state = FORMATION;
         }

         // Continue one step
         wb_robot_step(TIME_STEP);
    }
}


/*
 * Main function : receives the weight from supervisor, send them to the simulation and send back a signal to the supervisor when the simulation end
*/
int main() {
    // Init
    double end_sim[255];
    double *new_weights;
    double fitness=-1;
    reset();
    wb_motor_set_velocity(left_motor, 0);  // Initialize robot velocity to zero
    wb_motor_set_velocity(right_motor, 0); // Initialize robot velocity to zero

    // Loop forever
    while (1) {
        // Wait for data
        while (wb_receiver_get_queue_length(rec_pso) == 0) {
            wb_robot_step(TIME_STEP);
        }

        // Print weight
        if( 1 && robot_verbose){printf("************************ Weight ***************************\n");}
        new_weights = (double *)wb_receiver_get_data(rec_pso); // receive weight from supervisor
        if( 1 && robot_verbose){printf("Robot %d : \n %.2lf, %.2lf, %.2lf, %.4lf\n----------------------------\n",robot_id_u, new_weights[0], new_weights[1],new_weights[2], new_weights[3]);}

        // Run simulation
        simulation_webot(new_weights);
        end_sim[0]=fitness;

        // Send signal to supervisor
        wb_emitter_send(emit_pso,(void *)end_sim,sizeof(double));
        wb_receiver_next_packet(rec_pso);

        if( 1 && robot_verbose){printf("********************************************************\n");}
    }
    return 0;
}
