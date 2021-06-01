/*****************************************************************************/
/* File:         crossing_leader.c                                           */
/* Version:      1.0                                                         */
/* Date:         06-Jun-21                                                   */
/* Description:  Formation with relative positions in a world with           */
/*               two teams of robots crossing : leader controller            */
/*                                                                           */
/*****************************************************************************/
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <float.h>


#include "../localization_controller/utils.h"
#include "../localization_controller/odometry.h"
#include "../localization_controller/kalman.h"


#include <webots/robot.h>
/*Webots 2018b*/
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/position_sensor.h>
/*Webots 2018b*/
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define NB_SENSORS          8          // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SPEED         800     // Maximum speed
/*Webots 2018b*/
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
/*Webots 2018b*/
#define TIME_STEP      64      // [ms] Length of time step
#define AVOIDANCE_THRESH    1000  // Threshold above which we enter obstacle avoidance
#define WHEEL_AXIS        0.057        // Distance between the two wheels in meter
#define WHEEL_RADIUS            0.020        // Radius of the wheel in meter

// two leaders
#define LEADER1_ID 0  // leader ID of robots_ID_group1
#define LEADER2_ID 5  // leader ID of robots_ID_group2

//States of FSM
#define AVOIDANCE 0
#define MIGRATION 1
#define MIGRATION_WEIGHT  1   // Weight of attraction towards the common goal

/*MACRO*/
#define CATCH(X, Y)      X = X || Y
#define CATCH_ERR(X, Y)  controller_error(X, Y, __LINE__, __FILE__)
static FILE *fp;

/*VARIABLES*/
static bool controller_init_log(const char *filename);

static bool controller_init();

static void controller_print_log(double time);

static bool controller_error(bool test, const char *message, int line, const char *fileName);

double migrLeft[2] = {1.85, 0};
double migrRight[2] = {1.85, 0};
static pose_t _pose, _odo_enc, _kal_wheel;

// Initial robot position
static pose_t _pose_origin_R = {-0.06, 0.0, M_PI};
static pose_t _pose_origin_L = {-1.91, 0.0, 0};
static double speed[2];                 // Speed calculated for migration


/*Webots 2018b*/
WbDeviceTag dev_gps; // GPS handler
WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag left_encoder;//handler for left encoder of the robot
WbDeviceTag right_encoder;//handler for right encoder of the robot

int e_puck_matrix[16] = {17, 29, 34, 10, 8, -38, -56, -76, -72, -58, -36, 8, 10, 36, 28, 18}; // for obstacle avoidance

WbDeviceTag ds[NB_SENSORS];    // Handle for the infrared distance sensors
WbDeviceTag receiver;        // Handle for the receiver node
WbDeviceTag emitter;        // Handle for the emitter node

int robot_id_u;                // Unique robot ID
char *robot_name;

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
    wb_gps_enable(dev_gps, 1000); // Enable GPS every 1000ms <=> 1s
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
    char s[4] = "ps0";
    for (i = 0; i < NB_SENSORS; i++) {
        ds[i] = wb_robot_get_device(s);    // the device name is specified in the world file
        s[2]++;                // increases the device number
    }
    robot_name = (char *) wb_robot_get_name();


    for (i = 0; i < NB_SENSORS; i++) {
        wb_distance_sensor_enable(ds[i], 64);
    }

    wb_receiver_enable(receiver, 64);

    //Reading the robot's name.
    sscanf(robot_name, "epuck%d", &robot_id_u); // read robot id from the robot's name

    printf("Reset: robot %d\n", robot_id_u);
}


/*
 * Keep given int numbers within interval {-limit, limit} and rescale both numbers to keep their relative differences
 * If above limit, set biggest number to limit and the smallest number to (limit - their initial difference)
 */
void limit_and_rescale(int *number1, int *number2, int limit) {

    int delta;

    if ((*number1 > 0) && (*number2 > 0)) {

        if (*number1 > *number2) {

            delta = *number1 - *number2;
            if (*number1 > limit) {
                *number1 = limit;
                *number2 = limit - delta;
            }

        } else {

            delta = *number2 - *number1;
            if (*number2 > limit) {
                *number2 = limit;
                *number1 = limit - delta;
            }
        }
    } else if ((*number1 > 0) && (*number2 < 0)) {

        delta = *number1 - *number2;

        if ((*number1 >= limit) && (*number2 <= -limit)) {
            *number1 = limit;
            *number2 = -limit;
        } else if ((*number1 >= limit) && (*number2 > -limit)) {
            *number1 = limit;
            *number2 = limit - delta;
        } else if ((*number1 < limit) && (*number2 <= -limit)) {
            *number1 = -limit + delta;
            *number2 = -limit;
        }

    } else if ((*number1 < 0) && (*number2 > 0)) {

        delta = *number2 - *number1;

        if ((*number1 <= -limit) && (*number2 >= limit)) {
            *number1 = -limit;
            *number2 = limit;
        } else if ((*number1 > -limit) && (*number2 >= limit)) {
            *number1 = limit - delta;
            *number2 = limit;
        } else if ((*number1 <= -limit) && (*number2 < limit)) {
            *number1 = -limit;
            *number2 = -limit + delta;
        }

    } else if ((*number1 < 0) && (*number2 < 0)) {

        if (*number1 < *number2) {

            delta = *number2 - *number1;
            if (*number1 < -limit) {
                *number1 = -limit;
                *number2 = -limit + delta;
            }

        } else {

            delta = *number1 - *number2;
            if (*number2 < -limit) {
                *number1 = -limit + delta;
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

    float Ku = 0.2;   // Forward control coefficient
    float Kw = 0.5;  // Rotational control coefficient
    float range = sqrtf(x * x + z * z);      // Distance to the wanted position
    float bearing = atan2(z, x);      // Orientation of the wanted position

    // Compute forward control
    float u = Ku * range * cosf(bearing);
    // Compute rotational control
    float w = Kw * bearing;

    // Convert to wheel speeds!
    *msl = (u - WHEEL_AXIS * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    *msr = (u + WHEEL_AXIS * w / 2.0) * (1000.0 / WHEEL_RADIUS);
    limit_and_rescale(msl, msr, MAX_SPEED);
}


/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/
void send_ping(void) {
    char out[10];
    strcpy(out, robot_name);  // in the ping message we send the name of the robot.
    wb_emitter_send(emitter, out, strlen(out) + 1);
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

        inbuffer = (char *) wb_receiver_get_data(receiver);
        message_direction = wb_receiver_get_emitter_direction(receiver);
        message_rssi = wb_receiver_get_signal_strength(receiver);
        double z = message_direction[2];
        double x = message_direction[0];

        theta = -atan2(z, x);
        bearing = theta - M_PI / 2; // find the relative theta;
        range = sqrt((1 / message_rssi));

        if (bearing < 0) {// keep bearing between 0 and 2pi
            bearing = bearing + 2 * M_PI;
        }

        friend_robot_id = (int) (inbuffer[5] - '0');  // since the name of the sender is in the received message.


        // save the measured range and bearing if the sender is the other leader
        if ((friend_robot_id == LEADER1_ID) || (friend_robot_id == LEADER2_ID)) {

            range_to_other_leader = range;
            bearing_to_other_leader = bearing;

            //printf("Robot ID %d, my range %f and bearing %f to the other leader %d\n ", robot_id_u, range, bearing, friend_robot_id);
        }

        wb_receiver_next_packet(receiver);
    }
}


// the main function
int main() {


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
    if (CATCH_ERR(controller_init(), "Controller fails to init \n"))
        return 1;

    // initial state: migartion
    fsm_state = MIGRATION;



    // Forever
    for (;;) {
        // Get Position using Kalman
        time_step = wb_robot_get_basic_time_step();

        // Position with frame initial point stored in _pose vector
        controller_get_pose_gps();

        // Get the encoder values (wheel motor values)
        controller_get_encoder();


        /// Compute position from wheel encoders
        odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc,
                             _meas.right_enc - _meas.prev_right_enc);

        int time_step_ = wb_robot_get_basic_time_step();


        double time_now_s = wb_robot_get_time();

         printf("ODO_x = %g, ODO_y =%g, ODO_heading =%g\n", _odo_enc.x, _odo_enc.y, RAD2DEG(_odo_enc.heading));
        // Kalman with wheel encoders
        printf("pose_x = %g, pose_y =%g, pose_heading =%g\n", _pose.x, _pose.y, RAD2DEG(_pose.heading));
        compute_kalman_wheels(&_kal_wheel, time_step_, time_now_s, _meas.left_enc - _meas.prev_left_enc,
                              _meas.right_enc - _meas.prev_right_enc, _pose);
        printf("_kal_wheel_x = %g, _kal_wheel_y =%g, _kal_wheel_heading =%g\n", _kal_wheel.x, _kal_wheel.y, RAD2DEG(_kal_wheel.heading));


        bmsl = 0;
        bmsr = 0;
        sum_sensors = 0;
        max_sens = 0;

        // Braitenberg
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

        send_ping();  // sending a ping to other robot, so they can measure their distance to this robot
        process_received_ping_messages();



        // Condition for entering obstacle avoidance state (threshold on one of the four front sensors)
        if ((ds_value[0] > AVOIDANCE_THRESH ||
             ds_value[7] > AVOIDANCE_THRESH ||
             ds_value[6] > AVOIDANCE_THRESH ||
             ds_value[1] > AVOIDANCE_THRESH) && (fsm_state == MIGRATION)) {

            fsm_state = AVOIDANCE;
        }


        if (fsm_state == MIGRATION) { //formation state --- do only formation

            // This is the leader robot wanting to go left

            if (robot_id_u == 0) {


                double tmp_x = (migrLeft[0] - _kal_wheel.x);
                double tmp_z = (migrLeft[1] - _kal_wheel.y);

                speed[0] = tmp_x * MIGRATION_WEIGHT;
                speed[1] = tmp_z * MIGRATION_WEIGHT;


            } else if (robot_id_u == 5) {

                double tmp_x = (migrRight[0] - _kal_wheel.x);
                double tmp_z = (migrRight[1] - _kal_wheel.y);

                speed[0] = tmp_x * MIGRATION_WEIGHT;
                speed[1] = tmp_z * MIGRATION_WEIGHT;


            }

            compute_wheel_speeds(&msl, &msr);

        } else {
            //avoidance state --- do only braitenberg

            msl = bmsl;
            msr = bmsr;
        }

        limit_and_rescale(&msl, &msr, MAX_SPEED);

        // Set speed
        msl_w = msl * MAX_SPEED_WEB / (MAX_SPEED + 1);
        msr_w = msr * MAX_SPEED_WEB / (MAX_SPEED + 1);

        wb_motor_set_velocity(left_motor, msl_w);
        wb_motor_set_velocity(right_motor, msr_w);

        //Condition to exit avoidance state
        if ((fsm_state == AVOIDANCE) && (max_sens < 70)) { //Exit condition of "avoidance
            fsm_state = MIGRATION;
        }


        controller_print_log(wb_robot_get_time());

        // Continue one step --> change robot
        wb_robot_step(TIME_STEP);
    }
    if (fp != NULL)
        fclose(fp);
}


// Functions used for Kalman

void controller_get_pose_gps() {

    double time_now_s = wb_robot_get_time();


    if (time_now_s - last_gps_time_s > 1.0f) {
               // Update gps measurements
                       // Position from GPS
        controller_get_gps();
        

        last_gps_time_s = time_now_s;
        // This is for the robot which is starting
        if (robot_id_u == 0) {

            _pose.x = -(_meas.gps[0] - _pose_origin_R.x);

            _pose.y = (_meas.gps[2] - _pose_origin_R.y);

            //_pose.x = (_meas.gps[0] - _pose_origin_R.x)*cosf(_pose_origin_R.heading) +(_meas.gps[2] - _pose_origin_R.y)*sinf(_pose_origin_R.heading);

            //_pose.y = (_meas.gps[0] - _pose_origin_R.x)*sinf(_pose_origin_R.heading) +(_meas.gps[2] - _pose_origin_R.y)*cosf(_pose_origin_R.heading);

            _pose.heading = -controller_get_heading_gps() + _pose_origin_R.heading;
        } else if (robot_id_u == 5) {
            _pose.x = _meas.gps[0] - _pose_origin_L.x;

            _pose.y = -(_meas.gps[2] - _pose_origin_L.y);

            _pose.heading = -controller_get_heading_gps() + _pose_origin_L.heading;


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

    //_meas.gps_heading[0] = heading;
    printf("Heading GPS =%g\n", RAD2DEG(heading));

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

        ;
    }

    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;


    _meas.right_enc = wb_position_sensor_get_value(right_encoder);

    if (isnan(_meas.right_enc)) {
        _meas.right_enc = 0.0;

        ;
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
 *
 * @brief Printing onto log file variables of interest
 */
void controller_print_log(double time) {

    if (fp != NULL) {
        fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g\n",
                time, _pose.x, _pose.y, _pose.heading, _meas.gps[0], _meas.gps[2],
                _kal_wheel.x,
                _kal_wheel.y, _kal_wheel.heading);
    }

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
