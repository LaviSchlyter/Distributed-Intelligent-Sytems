/*****************************************************************************************************************************/
/* File:         robot_flock.c                                                 */
/* Version:      1.0                                                           */
/* Date:         06-Jun-21                                                     */
/* Description:  Flocking with relative positions in a world with             */
/*               obstacles                                                     */
/*                                                                             */
/* Author:       Paco Mermoud and Lavinia Schlyter                             */
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
#include <webots/gps.h>
#include <webots/position_sensor.h>

#include "../localization_controller/utils.h"
#include "../localization_controller/odometry.h"
#include "../localization_controller/kalman.h"

#define NB_SENSORS         8      // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800    // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define FLOCK_SIZE           5      // Size of flock (to change if necessary)
#define TIME_STEP           64      // [ms] Length of time step

#define AXLE_LENGTH        0.052    // Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS    0.00628    // Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS        0.0205    // Wheel radius (meters)
#define DELTA_T        0.064    // Timestep (seconds)

// Reynolds
#define RULE1_THRESHOLD     0.2   // Threshold to activate aggregation rule.
#define RULE1_WEIGHT        (0.6/10)*1       // Weight of aggregation rule. 

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule.
#define RULE2_WEIGHT        (0.02/10)*1       // Weight of dispersion rule. 

/** 
  As we have seen in the lab, the constistency rule may cause some issues and 
  we thus set it to zero and use the migration urge 
**/
#define RULE3_WEIGHT        (1.0/10)*0   // Weight of consistency rule. 

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction


WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS]; // ps[NB_SENSORS]
WbDeviceTag receiver;        // Handle for the receiver node
WbDeviceTag emitter;        // Handle for the emitter node
WbDeviceTag left_encoder;//handler for left encoder of the robot
WbDeviceTag right_encoder;//handler for right encoder of the robot
WbDeviceTag dev_gps; // GPS handler

int e_puck_matrix[16] = {17, 29, 34, 10, 8, -60, -64, -84,
                         -80, -66, -62, 8, 10, 36, 28, 18}; // for obstacle avoidance

int robot_id_u, robot_id;    // Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_verbose;

float relative_pos[FLOCK_SIZE][3];    // relative X, Z, Theta (heading)
float prev_relative_pos[FLOCK_SIZE][3];    // Previous relative  X, Z, Theta values
float my_position[3];            // X, Z, Theta of the current robot
float prev_my_position[3];        // X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];        // Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];    // Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];        // != 0 if initial positions have been received
float migr[2] = {0, -25};            // Migration vector
char *robot_name;
int avoidance = 0; // boolean to avoid obstacle
int side = 0; // side of avoidance (left=0, right=1)
float theta_robots[FLOCK_SIZE];


static double time_step;                  // Time step
static measurement_t _meas;               // See class in util
static double last_gps_time_s = 0.0f;
// Initial robot positions (x, theta, y) --> in pose (x,y,heading)
static pose_t _pose_origin_robot_0 = {-2.9, 0.0, 0};
static pose_t _pose_origin_robot_1 = {-2.9, 0.1, 0};
static pose_t _pose_origin_robot_2 = {-2.9, -0.1, 0};
static pose_t _pose_origin_robot_3 = {-2.9, 0.2, 0};
static pose_t _pose_origin_robot_4 = {-2.9, -0.2, 0};

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
    wb_gps_enable(dev_gps, 1000); // Enable GPS every 1000ms <=> 1s
    time_step = wb_robot_get_basic_time_step();
    receiver = wb_robot_get_device("receiver");
    emitter = wb_robot_get_device("emitter");

    //Get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);
    // Get wheel encoders
    left_encoder = wb_robot_get_device("left wheel sensor");
    right_encoder = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(left_encoder, TIME_STEP);
    wb_position_sensor_enable(right_encoder, TIME_STEP);


    int i;
    char s[4] = "ps0";
    for (i = 0; i < NB_SENSORS; i++) {
        ds[i] = wb_robot_get_device(s);    // the device name is specified in the world file
        s[2]++;                // increases the device number
    }
    robot_name = (char *) wb_robot_get_name();


    for (i = 0; i < NB_SENSORS; i++)
        wb_distance_sensor_enable(ds[i], TIME_STEP);

    wb_receiver_enable(receiver, TIME_STEP / 2);

    //Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
    sscanf(robot_name, "epuck%d", &robot_id_u); // read robot id from the robot's name
    robot_id = robot_id_u % FLOCK_SIZE;      // normalize between 0 and FLOCK_SIZE-1

    if (robot_id == 0) { // print robot data (ENLEVER)
        robot_verbose = 1;
    }

    for (i = 0; i < FLOCK_SIZE; i++) {
        initialized[i] = 0;          // Set initialization to 0 (= not yet initialized)
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
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) {

    float dr = (float) msr * SPEED_UNIT_RADS * DELTA_T; //radians
    float dl = (float) msl * SPEED_UNIT_RADS * DELTA_T;
    double time_now_s = wb_robot_get_time();


    // Position from GPS
    // Position with frame initial point stored in _pose vector
    controller_get_pose_gps();

    // Update odometry with wheel encoders usng pose (GPS derived)
    compute_kalman_wheels(&_kal_wheel, TIME_STEP, time_now_s, dl,
                          dr, _pose);

    my_position[0] = _kal_wheel.x;
    my_position[1] = _kal_wheel.y;
    my_position[2] = _kal_wheel.heading;

}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) {
    // Compute wanted position from Reynold's speed and current location
    float x = speed[robot_id][0] * cosf(my_position[2]) +
              speed[robot_id][1] * sinf(my_position[2]); // x in robot coordinates
    float z = -speed[robot_id][0] * sinf(my_position[2]) +
              speed[robot_id][1] * cosf(my_position[2]); // z in robot coordinates

    float Ku = 0.2;   // Forward control coefficient
    float Kw = 0.5;  // Rotational control coefficient
    float range = sqrtf(x * x + z * z);      // Distance to the wanted position
    float bearing = -atan2(x, z);      // Orientation of the wanted position

    // Compute forward control
    float u = Ku * range * cosf(bearing);
    // Compute rotational controlcontroller_get_pose_gps
    float w = Kw * bearing;

    // Convert to wheel speeds!
    *msl = (u - AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + AXLE_LENGTH * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    limit(msl, MAX_SPEED);
    limit(msr, MAX_SPEED);
}


/*
 *  Update speed according to Reynold's rules
 */
void reynolds_rules() {

    int i, j, k;            // Loop counters
    float rel_avg_loc[2] = {0, 0};    // Flock average positions
    float rel_avg_speed[2] = {0, 0};    // Flock average speeds
    float cohesion[2] = {0, 0};
    float dispersion[2] = {0, 0};
    float consistency[2] = {0, 0};


    /* Compute averages over the whole flock */
    for (i = 0; i < FLOCK_SIZE; i++) {
        if (i == robot_id)
            continue; // don't consider yourself for the average
        for (j = 0; j < 2; j++) {
            rel_avg_speed[j] += relative_speed[i][j];
            rel_avg_loc[j] += relative_pos[i][j];
        }
    }
    for (j = 0; j < 2; j++) {
        rel_avg_speed[j] /= FLOCK_SIZE - 1;
        rel_avg_loc[j] /= FLOCK_SIZE - 1;
    }

    /* Rule 1 - Aggregation/Cohesion: move towards the center of mass */

    for (j = 0; j < 2; j++) {

        if (sqrt(pow(rel_avg_loc[0], 2) + pow(rel_avg_loc[1], 2)) > RULE1_THRESHOLD) {
            cohesion[j] = rel_avg_loc[j];
        }

    }

    /* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
    for (k = 0; k < FLOCK_SIZE; k++) {
        if (k != robot_id) {        // Loop on flockmates only
            // If neighbor k is too close (Euclidean distance)
            if (sqrt(pow(relative_pos[k][0], 2) + pow(relative_pos[k][1], 2)) < RULE2_THRESHOLD) {
                for (j = 0; j < 2; j++) {
                    dispersion[j] -= 1 / (relative_pos[k][j]);    // Relative distance to robot k
                }
            }
        }
    }

    /* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
    for (j = 0; j < 2; j++) {
        consistency[j] = rel_avg_speed[j];
    }

    //aggregation of all behaviors with relative influence determined by weights
    for (j = 0; j < 2; j++) {
        speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
        speed[robot_id][j] += dispersion[j] * RULE2_WEIGHT;
        speed[robot_id][j] += consistency[j] * RULE3_WEIGHT;

    }
    speed[robot_id][1] *= -1; //y axis of webots is inverted

    //move the robot according to some migration rule
    if (MIGRATORY_URGE == 0) {
        speed[robot_id][0] += 0.01 * cos(my_position[2] + M_PI / 2);
        speed[robot_id][1] += 0.01 * sin(my_position[2] + M_PI / 2);
    } else {

        speed[robot_id][0] += (migr[0] - my_position[0]) * MIGRATION_WEIGHT;
        speed[robot_id][1] -= (migr[1] - my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted

    }
}


/*
 *  Each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/

void send_ping(void) {
    char out[10];
    strcpy(out, robot_name);  // in the ping message we send the name of the robot.
    wb_emitter_send(emitter, out, strlen(out) + 1);
}

/*
 * Processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void) {
    const double *message_direction;
    double message_rssi; // Received Signal Strength indicator
    double theta;
    double range;
    char *inbuffer;    // Buffer for the receiver node
    int other_robot_id;
    while (wb_receiver_get_queue_length(receiver) > 0) {
        inbuffer = (char *) wb_receiver_get_data(receiver);
        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);
        double y = message_direction[2];
        double x = message_direction[0];

        theta = -atan2(y, x);// Estimated theta
        theta = theta + my_position[2]; // find the relative theta;
        range = sqrt((1 / message_rssi));

        // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!
        other_robot_id = (int) (inbuffer[5] -
                                '0');  

        // Get position update
        prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
        prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

        relative_pos[other_robot_id][0] = range * cos(theta);  // relative x pos
        relative_pos[other_robot_id][1] = -1.0 * range * sin(theta);   // relative y pos

        relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0] * 0.0 + 1.0 * (1 / DELTA_T) *
                                                                                      (relative_pos[other_robot_id][0] -
                                                                                       prev_relative_pos[other_robot_id][0]);
        relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1] * 0.0 + 1.0 * (1 / DELTA_T) *
                                                                                      (relative_pos[other_robot_id][1] -
                                                                                       prev_relative_pos[other_robot_id][1]);

        wb_receiver_next_packet(receiver);
    }
}


// the main function
int main() {
    int msl, msr;            // Wheel speeds
    float msl_w, msr_w;
    int bmsl, bmsr, sum_sensors;    // Braitenberg parameters
    int i;                // Loop counter
    int ds_value[NB_SENSORS];    // Array for the distance sensor readings
    int max_sens;            // Store highest sensor value
    float priority; // Non linear weight to prioritarize braitenberg
    reset();            // Resetting the robot
    msl = 0;
    msr = 0;
    max_sens = 0;

    // Forever
    for (;;) {

        bmsl = 0;
        bmsr = 0;
        sum_sensors = 0;
        max_sens = 0;

        /* Braitenberg */
        for (i = 0; i < NB_SENSORS; i++) {
            ds_value[i] = wb_distance_sensor_get_value(ds[i]); //Read sensor values
            sum_sensors += ds_value[i]; // Add up sensor values
            max_sens = max_sens > ds_value[i] ? max_sens : ds_value[i]; // Check if new highest sensor value

            // Weighted sum of distance sensor values for Braitenburg vehicle
            bmsr += e_puck_matrix[i] * ds_value[i];
            bmsl += e_puck_matrix[i + NB_SENSORS] * ds_value[i];
        }


        // Adapt Braitenberg values (empirical tests)
        bmsl /= MIN_SENS;
        bmsr /= MIN_SENS;
        bmsl += 66;
        bmsr += 72;

        /* Send and get information */
        // sending a ping to other robot, so they can measure their distance to this robot
        send_ping();  

        /// Compute self position
        prev_my_position[0] = my_position[0];
        prev_my_position[1] = my_position[1];

        // Update position using Kalman and odometry
        update_self_motion(msl, msr);

        process_received_ping_messages();


        speed[robot_id][0] = (1 / DELTA_T) * (my_position[0] - prev_my_position[0]);
        speed[robot_id][1] = (1 / DELTA_T) * (my_position[1] - prev_my_position[1]);

        // Reynold's rules with all previous info (updates the speed[][] table)
        reynolds_rules();

        // Compute wheels speed from reynold's speed
        compute_wheel_speeds(&msl, &msr);

        priority = 2;
        if ((ds_value[0] > 2100 || ds_value[7] > 2100) && !avoidance) { //Entry1 condition of "avoidance
            avoidance = 1;
            // See whether to avoid to the left or to the right
            if (ds_value[7] < ds_value[0]) {
                side = 0; // left

            } else {
                side = 1; // right

            }

        } else if ((ds_value[6] > 2100 || ds_value[1] > 2100) && !avoidance) { //Entry2 condition of "avoidance
            avoidance = 1;
            // See whether to avoid to the left or to the right
            if (ds_value[6] < ds_value[1]) {
                side = 0; // left

            } else {
                side = 1; // right

            }
        } else if ((ds_value[5] > 2100 || ds_value[2] > 2100) && !avoidance) { //Entry2 condition of "avoidance
            avoidance = 1;
            // See whether to avoid to the left or to the right
            if (ds_value[5] < ds_value[2]) {
                side = 0; // left

            } else {
                side = 1; // right

            }


        }


        if (!avoidance) { // normal mode
            msl -= msl * max_sens / (priority * MAX_SENS);
            msr -= msr * max_sens / (priority * MAX_SENS);
        } else { // avoidance mode
            if (side) { // avoid to the right
                if (ds_value[7] > 1300) {
                    msl = 140;
                    msr = 150;
                } else {
                    msl = 100;
                    msr = 150;

                }

            } else { // avoid to the left
                if (ds_value[0] > 1200) {
                    msl = 150;
                    msr = 140;
                } else {

                    msl = 150;
                    msr = 100;

                }
            }
        }

        if (avoidance && max_sens < 70) { //Exit condition of "avoidance
            avoidance = 0;
        }


        // Add Braitenberg

        msl += bmsl;
        msr += bmsr;
        limit(&msl, MAX_SPEED);
        limit(&msr, MAX_SPEED);

        // Set speed
        msl_w = msl * MAX_SPEED_WEB / (MAX_SPEED + 1);
        msr_w = msr * MAX_SPEED_WEB / (MAX_SPEED + 1);

        wb_motor_set_velocity(left_motor, msl_w);
        wb_motor_set_velocity(right_motor, msr_w);

        // Continue one step
        wb_robot_step(TIME_STEP);
    }

}

/// Functions used for computing the position with Kalman

/**
 * @brief Compute the robot pose using 1 second GPS interval values
 * Because the position of origin (start) are different for each robot the pose values are different for each 
*/

void controller_get_pose_gps() {

    double time_now_s = wb_robot_get_time();

    if (time_now_s - last_gps_time_s > 1.0f) {
        controller_get_gps();

        last_gps_time_s = time_now_s;

        // Each robot has a different starting point 
        if (robot_id_u == 0) {

            _pose.x = _meas.gps[0] - _pose_origin_robot_0.x;

            _pose.y = -(_meas.gps[2] - _pose_origin_robot_0.y);

            _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_0.heading;

        } else if (robot_id_u == 1) {
            _pose.x = _meas.gps[0] - _pose_origin_robot_1.x;

            _pose.y = -(_meas.gps[2] - _pose_origin_robot_1.y);

            _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_1.heading;

        } else if (robot_id_u == 2) {
            _pose.x = _meas.gps[0] - _pose_origin_robot_2.x;

            _pose.y = -(_meas.gps[2] - _pose_origin_robot_2.y);

            _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_2.heading;

        } else if (robot_id_u == 3) {
            _pose.x = _meas.gps[0] - _pose_origin_robot_3.x;

            _pose.y = -(_meas.gps[2] - _pose_origin_robot_3.y);

            _pose.heading = -controller_get_heading_gps() + _pose_origin_robot_3.heading;

        } else if (robot_id_u == 4) {
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
        _meas.left_enc = 0.0;}

    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;

    _meas.right_enc = wb_position_sensor_get_value(right_encoder);
    if (isnan(_meas.right_enc)) {
        _meas.right_enc = 0.0;}
}


