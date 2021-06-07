/*****************************************************************************************************************************/
/* File:         pso_leader_formation.c                                        */
/* Version:      1.0                                                           */
/* Date:         01-Mai-21                                                     */
/* Description:  PSO for the obstacle formation (laplacian) with a group       */
/*               public homogeneous configuration on the followers.             */
/*                                                                             */
/* Author:       Paco Mermoud                                                  */
/*****************************************************************************************************************************/

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "../localization_controller/utils.h"
#include "../localization_controller/odometry.h"
#include "../localization_controller/kalman.h"

#include <webots/robot.h>
#include <webots/gps.h>
#include <webots/position_sensor.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS	       8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SPEED         800     // Maximum speed
#define MAX_SPEED_WEB     6.28    // Maximum speed Webots
#define TIME_STEP	       64	  // Length of time step in [ms]
#define AVOIDANCE_THRESH  1800    // Threshold above which we enter obstacle avoidance
#define MIGRATION_THRESH    70    // Threshold under which we enter migration state
#define WHEEL_AXIS       0.057    // Distance between the two wheels in meter
#define WHEEL_RADIUS     0.020    // Radius of the wheel in meter

//States of FSM
#define AVOIDANCE 0
#define MIGRATION 1

// You can control the speed here
#define MIGRATION_WEIGHT  0.003   // Weight of attraction towards the common goal

// PSO  -------------------- Choose the Sim step -----------------------------------------------------------------
#define SIM_STEPS 2000                  // Number of simulation steps/iterations (Change it also in Leader)
#define DATASIZE NB_SENSORS+4           // Number of elements in particle

/*MACRO*/
#define CATCH(X, Y)      X = X || Y
#define CATCH_ERR(X, Y)  controller_error(X, Y, __LINE__, __FILE__)
static FILE *fp;

/*FUNCTIONS*/
static void controller_get_pose_gps();
static void controller_get_gps();
static double controller_get_heading_gps();
static void controller_get_encoder();
static bool controller_init_log(const char *filename);
static bool controller_init();
static bool controller_error(bool test, const char *message, int line, const char *fileName);

WbDeviceTag dev_gps;        // Handle for GPS
WbDeviceTag left_motor;     // Handle for left wheel of the robot
WbDeviceTag right_motor;    // Handle for the right wheel of the robot
WbDeviceTag left_encoder;   // Handle for left encoder of the robot
WbDeviceTag right_encoder;  // Handle for right encoder of the robot
WbDeviceTag ds[NB_SENSORS]; // Handle for the infrared distance sensors
WbDeviceTag receiver;       // Handle for the receiver node
WbDeviceTag emitter;        // Handle for the emitter node
WbDeviceTag rec_pso;		// Handle for the receiver node for PSO
WbDeviceTag emit_pso;		// Handle for the emitter node for PSO

int robot_id_u;                  // Unique robot ID
char* robot_name;
int robot_verbose=0;             // boolean to know for which robot we print

int fsm_state;                   // leader's state of the FSM (avoidance or migration)
static double time_step;         // Time step
static measurement_t _meas;      // See class in utile
static double speed[2];          // Speed calculated for migration

static double last_gps_time_s = 0.0f;
static pose_t _pose, _kal_wheel;
int e_puck_matrix[2*NB_SENSORS] = {17,29,34,10,8,-38,-56,-76,-72,-58,-36,8,10,36,28,18}; // for obstacle avoidance

// Initial robot position
static pose_t _pose_origin = {-2.8, 0.0, 0.0};
// Migration target
double migr[2] = {4.4, -0.45};


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

    //get encoders
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
        s[2]++;			// increases the device number
    }
    robot_name=(char*) wb_robot_get_name();

    for(i=0;i<NB_SENSORS;i++){
        wb_distance_sensor_enable(ds[i],64);
    }

    wb_receiver_enable(receiver,64);

    //Reading the robot's name.
    sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
    if(robot_id_u==0){ // print robot data (ENLEVER)
        robot_verbose=1;
    }

    // Receiver and emitter PSO
    emit_pso = wb_robot_get_device("emitter_epuck_pso");
    rec_pso = wb_robot_get_device("receiver_epuck_pso");
    wb_receiver_enable(rec_pso, TIME_STEP/2);
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
void compute_wheel_speeds(int *msl, int *msr) {
    // Compute wanted position from Reynold's speed and current location

    float x = speed[0] * cosf(_kal_wheel.heading) + speed[1] * sinf(_kal_wheel.heading); // x in robot coordinates
    float z = -speed[0] * sinf(_kal_wheel.heading) + speed[1] * cosf(_kal_wheel.heading); // z in robot coordinate

    float Ku = 0.2;
    float Kw = 0.5;
    float range = sqrtf(x * x + z * z);
    float bearing = atan2(z, x);

    float u = Ku * range * cosf(bearing);
    float w = Kw * bearing;

    *msl = (u - WHEEL_AXIS * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + WHEEL_AXIS * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    limit_and_rescale(msl, msr, MAX_SPEED);
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
 * Compute the simulation for SIM_STEPS iteration
*/
int simulation_webot(double new_weights[DATASIZE]){

    int msl = 0;             // Left wheel speed
    int msr = 0;            // Right wheel speed
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int i,j;                        // Loop counter
    int ds_value[NB_SENSORS];       // Array for the distance sensor readings
    int max_sens;                   // Store highest sensor value
    double time_now_s;
    double tmp_x;
    double tmp_z;

    if (CATCH_ERR(controller_init(), "Controller fails to init \n")){
        return 1;
    }

    // initial state: migration
    fsm_state = MIGRATION;

    // Forever
    for(j=0;j<SIM_STEPS;j++){

        // Get Position using Kalman
        time_step = wb_robot_get_basic_time_step();

        // Position with frame initial point stored in _pose vector
        controller_get_pose_gps();

        // Get the encoder values (wheel motor values)
        controller_get_encoder();

        time_now_s = wb_robot_get_time();
        // Kalman with wheel encoders
        compute_kalman_wheels(&_kal_wheel, time_step, time_now_s, _meas.left_enc - _meas.prev_left_enc,_meas.right_enc - _meas.prev_right_enc, _pose);

        bmsl = 0;
        bmsr = 0;
        sum_sensors = 0;
        max_sens = 0;

        // Braitenberg
        for(i = 0; i < NB_SENSORS; i++) {
             ds_value[i] = wb_distance_sensor_get_value(ds[i]); //Read sensor values
             sum_sensors += ds_value[i]; // Add up sensor values
             max_sens = max_sens>ds_value[i]?max_sens:ds_value[i]; // Check if new highest sensor value
             // Weighted sum of distance sensor values for Braitenberg vehicle
             bmsr += e_puck_matrix[i] * ds_value[i];
             bmsl += e_puck_matrix[i + NB_SENSORS] * ds_value[i];
        }

        // Adapt Braitenberg values (empirical tests)
        bmsl/=MIN_SENS;
        bmsr/=MIN_SENS;
        bmsl+=66;
        bmsr+=72;

        send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

        // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
        if ((ds_value[0] > AVOIDANCE_THRESH ||
             ds_value[7] > AVOIDANCE_THRESH ||
             ds_value[6] > AVOIDANCE_THRESH ||
             ds_value[1] > AVOIDANCE_THRESH) && (fsm_state == MIGRATION)){

             fsm_state = AVOIDANCE;
        }

        if (fsm_state == MIGRATION){ //migration

             tmp_x = (migr[0] - _kal_wheel.x);
             tmp_z = (migr[1] - _kal_wheel.y);


             // to avoid slowing down too much when the leader is approaching the goal position
             if (tmp_x > 2){

                speed[0] = tmp_x * MIGRATION_WEIGHT;
                speed[1] = tmp_z * MIGRATION_WEIGHT;
             }
             else if ((tmp_x <= 2) && (tmp_x >= 1)){

                 speed[0] = tmp_x * (2*MIGRATION_WEIGHT);
                 speed[1] = tmp_z * (2*MIGRATION_WEIGHT);
             }
             else {
                 speed[0] = tmp_x * (4*MIGRATION_WEIGHT);
                 speed[1] = tmp_z * (4*MIGRATION_WEIGHT);
             }
             compute_wheel_speeds(&msl, &msr);
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
        if((fsm_state == AVOIDANCE) && (max_sens < MIGRATION_THRESH)){ //Exit condition of avoidance state
            fsm_state = MIGRATION;
        }

        // Continue one step --> on change de robot!
        wb_robot_step(TIME_STEP);
    }
    return 0;
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
    odo_reset(time_step);
    wb_motor_set_velocity(left_motor, 0);  // Initialize robot velocity to zero
    wb_motor_set_velocity(right_motor, 0); // Initialize robot velocity to zero

    // Loop forever
    while (1) {
        // Wait for data
        while (wb_receiver_get_queue_length(rec_pso) == 0) {
            wb_robot_step(TIME_STEP);
        }

        // Extract new weight from supervisor
        new_weights = (double *)wb_receiver_get_data(rec_pso);


        // Update initial position
        _pose.heading=0;
        _pose.y=0;
        _pose.x=0;
        _pose_origin.y=new_weights[DATASIZE];
        controller_get_pose_gps();
        kal_reset();
        odo_reset(TIME_STEP);

        // Run simulation
        fitness = simulation_webot(new_weights);
        end_sim[0]=fitness;

        // Send signal to supervisor
        wb_emitter_send(emit_pso,(void *)end_sim,sizeof(double));
        wb_receiver_next_packet(rec_pso);

        if( 1 && robot_verbose){printf("********************************************************\n");}
    }
    return 0;
}


/**
 *
 * @brief Determines the pose of the robot based on the GPS measurments
 */
void controller_get_pose_gps() {

    double time_now_s = wb_robot_get_time();
    if (time_now_s - last_gps_time_s > 1.0f) { // Update gps measurements

        controller_get_gps();
        last_gps_time_s = time_now_s;

        _pose.x = _meas.gps[0] - _pose_origin.x;
        _pose.y = -(_meas.gps[2] - _pose_origin.y);
        // not used in this part: just used for completeness purposes
        _pose.heading = -controller_get_heading_gps() + _pose_origin.heading;
    }
}


/**
 *
 * @brief Get the GPS measurements for position of robots
 */
void controller_get_gps() {

    // Stores in memory at address of _meas.prev_gps; the data of _meas.gps
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

    // If nan, it means that you are at the first itteration
    if (isnan(_meas.left_enc)) {
        _meas.left_enc = 0.0;
    }

    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;
    _meas.right_enc = wb_position_sensor_get_value(right_encoder);

    if (isnan(_meas.right_enc)) {
        _meas.right_enc = 0.0;
    }
}


/**
 * @brief      Initialize the logging of the file
 *
 * @param[in]  filename  The filename to write
 *
 * @return     return true if it fails
 */
bool controller_init_log(const char *filename) {
    fp = fopen(filename, "w");
    bool err = CATCH_ERR(fp == NULL, "Fails to create a log file\n");

    if (!err) {
        fprintf(fp,
                "time; pose_x; pose_y; pose_heading;  gps_x; gps_y; gps_z; acc_x; acc_y; acc_z; right_enc; left_enc; odo_acc_x; odo_acc_y; odo_acc_heading; odo_enc_x; odo_enc_y; odo_enc_heading; kal_wheel_x; kal_wheel_y; kal_wheel_heading; kal_acc_x; kal_acc_y; kal_acc_heading\n");
    }

    return err;
}


bool controller_init() {
    bool err = false;
    char filename[64];
    sprintf(filename, "log_file_robot%d.csv", robot_id_u);
    CATCH(err, controller_init_log(filename));
    return err;
}

/**
 * @brief      Do an error test if the result is true write the message in the stderr.
 *
 * @param[in]  test     The error test to run
 * @param[in]  message  The error message
 *
 * @return     true if there is an error
 */
bool controller_error(bool test, const char *message, int line, const char *fileName) {
    if (test) {
        char buffer[256];
        sprintf(buffer, "file : %s, line : %d,  error : %s", fileName, line, message);
        fprintf(stderr, buffer);

        return (true);
    }

    return false;
}
