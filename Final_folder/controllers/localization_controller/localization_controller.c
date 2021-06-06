/**************************************************************************************************************************/
/* File:         localization_controller.c                                         */
/* Version:      1.0                                                               */
/* Date:         06-Jun-21                                                         */
/* Description:  Computing position in the robot frame using odometry and kalman   */
/*               Pose update using GPS (on robot frame) starting at second =1      */
/*                                                                                 */
/**************************************************************************************************************************/


#include <stdio.h>
#include <string.h>
#include <math.h>

/*MACRO*/
#define CATCH(X, Y)      X = X || Y
#define CATCH_ERR(X, Y)  controller_error(X, Y, __LINE__, __FILE__)


#include "utils.h"
#include "odometry.h"
#include "kalman.h"
#include "trajectories.h"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

/*CONSTANTS*/
#define MAX_SPEED 1000          // Maximum speed
#define INC_SPEED 5             // Increment not expressed in webots
#define MAX_SPEED_WEB 6.28      // Maximum speed webots

/* TO COMPUTE ACCELERATION BIAS
If you would like to calibrate please:
-  define TIME_INIT_ACC 120 (for 120 seconds) else 0
- Set VERBOSE_CALIBRATION to true 
- pick the trajectory (3) in main
*/
// 
#define TIME_INIT_ACC 0           // Time in seconds
#define true 1
#define false 0
#define VERBOSE_CALIBRATION false // Set to true for calibrating the mean acceleration
#define VERBOSE_PRINT_LOG true    // Print log on CSV file
#define VERBOSE_ROBOT_POSE true        // Print the position of the robot updated each second

/*VARIABLES*/
static pose_t _pose, _odo_acc, _odo_enc, _kal_wheel, _kal_acc;

// Initial robot position
static pose_t _pose_origin = {-2.9, 0.0, 0};
static FILE *fp;

/*FUNCTIONS*/
static bool controller_init_log(const char *filename);

static bool controller_init();

static void controller_print_log(double time);

static bool controller_error(bool test, const char *message, int line, const char *fileName);

void init_devices(int ts);

static void controller_get_pose_gps();

static void controller_get_gps();

static double controller_get_heading_gps();

static void controller_get_acc();

static void controller_get_encoder();

static void controller_compute_mean_acc();

// Measurement structure with several variables (See class in util.h)
static measurement_t _meas; 
double last_gps_time_s = 0.0f;
int time_step; // Introduce the time step


WbDeviceTag dev_gps; //GPS Handler
WbDeviceTag dev_acc; //Acceleration Handler
// Encoder handler
WbDeviceTag dev_left_encoder; 
WbDeviceTag dev_right_encoder;
// Motor  handler
WbDeviceTag dev_left_motor;
WbDeviceTag dev_right_motor;



void init_devices(int ts) {
    dev_gps = wb_robot_get_device("gps");
    wb_gps_enable(dev_gps, 1000); // Enable GPS every 1000ms <=> 1s


    dev_acc = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable(dev_acc, ts); // Time step frequency (ts)

	
    // Handler for wheel encoders
    dev_left_encoder = wb_robot_get_device("left wheel sensor");
    dev_right_encoder = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(dev_left_encoder, ts);
    wb_position_sensor_enable(dev_right_encoder, ts);

    // Handler for wheel motor
    dev_left_motor = wb_robot_get_device("left wheel motor");
    dev_right_motor = wb_robot_get_device("right wheel motor");
    
    wb_motor_set_position(dev_left_motor, INFINITY);
    wb_motor_set_position(dev_right_motor, INFINITY);
    wb_motor_set_velocity(dev_left_motor, 0.0);
    wb_motor_set_velocity(dev_right_motor, 0.0);
    
}



int main() {

    wb_robot_init();

    if (CATCH_ERR(controller_init(), "Controller fails to init \n"))
        return 1;
    
    time_step = wb_robot_get_basic_time_step();
    init_devices(time_step);
    odo_reset(time_step);


    /// Mean accelerations found when calibrating projected onto world frame
    _meas.acc_mean[0] = -6.44938e-05; //y
    _meas.acc_mean[1] = 0.00766816; // x
    _meas.acc_mean[2] = 9.62942; // z

    // Forever
    while (wb_robot_step(time_step) != -1) {

        if (wb_robot_get_time() < TIME_INIT_ACC) {
        	// CALIBRATION
            controller_compute_mean_acc();
        } else {
        
         if (!VERBOSE_CALIBRATION) {

                
                
                // Updating the pose (thanks to GPS, every second)
                controller_get_pose_gps();
        
        
                // Get the acceleration from webots
                controller_get_acc();
        
                // Get the encoder values 
                controller_get_encoder();
                
                double time_now_s = wb_robot_get_time();
                
                
                /// Compute position from wheel encoders
                odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc,
                                     _meas.right_enc - _meas.prev_right_enc);

                /// Compute position from accelerometer with heading from wheel encoders
                odo_compute_acc(&_odo_acc, _meas.acc, _meas.acc_mean, _odo_enc.heading);

                // Current time in seconds
                time_now_s = wb_robot_get_time();

                
                // Kalman with accelerometer
                compute_kalman_acc(&_kal_acc, time_step, time_now_s, _odo_enc.heading, _meas, _pose);
                

                // Kalman with wheel encoders
                compute_kalman_wheels(&_kal_wheel, time_step, time_now_s, _meas.left_enc - _meas.prev_left_enc,
                                      _meas.right_enc - _meas.prev_right_enc, _pose);
                                      

            }

            /// Trajectories

            //trajectory_1(dev_left_motor, dev_right_motor);
            trajectory_2(dev_left_motor, dev_right_motor);
            
            // Trajectory 3 for accelerometer CALIBRATION
            //trajectory_3(dev_left_motor, dev_right_motor);


        }

        /// Printing the log file onto a csv for matlab plotting
        if (VERBOSE_PRINT_LOG) {
            controller_print_log(wb_robot_get_time());
        }
    }
    // Close log file
    if (fp != NULL)
        fclose(fp);
    // End of the simulation
    wb_robot_cleanup();

    return 0;

}

/**
 * @brief Compute the robot pose using 1 second GPS interval values
 */
void controller_get_pose_gps() {

    double time_now_s = wb_robot_get_time();


    if (time_now_s - last_gps_time_s > 1.0f) {
    
         // Update gps measurements every second 
         controller_get_gps();

        last_gps_time_s = time_now_s;

        _pose.x = _meas.gps[0] - _pose_origin.x;

	// Inverted world
        _pose.y = -(_meas.gps[2] - _pose_origin.y);
        
        _pose.heading = -controller_get_heading_gps() + _pose_origin.heading;
        
        if (VERBOSE_ROBOT_POSE) {
          printf("ROBOT pose : %g %g %g\n", _pose.x, _pose.y, RAD2DEG(_pose.heading));
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
 * @brief      Read the acceleration values from the sensor
 */
void controller_get_acc() {

    /// Call the function to get the accelerometer measurements.
    const double *acc_values = wb_accelerometer_get_values(dev_acc);

    /// Copy the acc_values into the measurment structure (static variable)
    memcpy(_meas.acc, acc_values, sizeof(_meas.acc));
}


/**
 * @brief      Read the encoders values from the sensors
 */
void controller_get_encoder() {
    // Store previous value of the left encoder
    _meas.prev_left_enc = _meas.left_enc;

    _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);

    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;

    _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

}

/**
 * @brief      Compute the mean of the 3-axis accelerometer. The result is stored in array _meas.acc
 */
void controller_compute_mean_acc() {

    static int count = 0;
    count++;

    // Remove the effects of strong acceleration at the beginning
    if (count > 20)
    {
        for (int i = 0; i < 3; i++) {
            _meas.acc_mean_calibration[i] += _meas.acc_mean[i];
        }
    }

    if (count == (int) ((TIME_INIT_ACC / (double) time_step) * 1000 - 1)) {
        printf("Accelerometer initialization Done ! \n");
        for (int i = 0; i < 3; i++) {
            _meas.acc_mean_calibration[i] = _meas.acc_mean_calibration[i] / (count-20);
        }

        printf("mean_Y = %g, meanx = %g, meanz = %g\n", _meas.acc_mean_calibration[0], _meas.acc_mean_calibration[1],
               _meas.acc_mean_calibration[2]);

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
    CATCH(err, controller_init_log("log_file.csv"));
    return err;
}

/**
 *
 * @brief Printing onto log file variables of interest
 */
void controller_print_log(double time) {

    if (fp != NULL) {
        fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g\n",
                time, _pose.x, _pose.y, _pose.heading, _meas.gps[0], _meas.gps[2],
                _meas.gps[1], _meas.acc[1] - _meas.acc_mean[1], _meas.acc[0] - _meas.acc_mean[0],
                _meas.acc[2] - _meas.acc_mean[2], _meas.right_enc, _meas.left_enc,
                _odo_acc.x, _odo_acc.y, _odo_acc.heading, _odo_enc.x, _odo_enc.y, _odo_enc.heading, _kal_wheel.x,
                _kal_wheel.y, _kal_wheel.heading, _kal_acc.x, _kal_acc.y, _kal_acc.heading);
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
