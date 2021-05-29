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
#include "localization_controller.h"

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/position_sensor.h>

/*CONSTANTES*/
#define MAX_SPEED 800          // Maximum speed
#define INC_SPEED 5             // Increment not expressed in webots
#define MAX_SPEED_WEB 6.28      // Maximum speed webots
#define FALSE 0
#define VERBOSE_PRINT_LOG TRUE    // Print log on CSV file
#define VERBOSE_ROBOT_POSE TRUE        // Print the position of the robot updated each second

static pose_t _pose, _odo_enc, _kal_wheel;
static measurement_t _meas; // See class in util
double last_gps_time_s = 0.0;

static FILE *fp;
//static bool controller_init_log(const char *filename);
//static bool controller_init();
//static void controller_print_log(double time);
//static bool controller_error(bool test, const char *message, int line, const char *fileName);

WbDeviceTag dev_gps;
WbDeviceTag dev_left_encoder;
WbDeviceTag dev_right_encoder;



void init_devices(int ts) {
    dev_gps = wb_robot_get_device("gps");
    wb_gps_enable(dev_gps, 1000); // Enable GPS every 1000ms <=> 1s
    
    dev_left_encoder = wb_robot_get_device("left wheel sensor");
    dev_right_encoder = wb_robot_get_device("right wheel sensor");
    wb_position_sensor_enable(dev_left_encoder, ts);
    wb_position_sensor_enable(dev_right_encoder, ts);    
}

/*FUNCTIONS*/
// Make static to limit its scope

//static void controller_get_pose_gps();

//static void controller_get_gps();

//static double controller_get_heading_gps();

//static void controller_get_encoder(); 

void compute_localization(pose_t* _pose_origin) {
    

//wb_robot_init();
// if (CATCH_ERR(controller_init(), "Controller fails to init \n"))
  
double time_step = wb_robot_get_basic_time_step();
init_devices(time_step);
odo_reset(time_step);

	// Position from GPS
	// Position with frame initial point stored in _pose vector
	controller_get_pose_gps(_pose_origin);

	// Get the encoder values (wheel motor values)
	controller_get_encoder();

        // Update gps measurements
        controller_get_gps();
        
        /// Compute position from wheel encoders
        odo_compute_encoders(&_odo_enc, _meas.left_enc - _meas.prev_left_enc,
                             _meas.right_enc - _meas.prev_right_enc);

        double time_now_s = wb_robot_get_time();
        int time_step_ = wb_robot_get_basic_time_step();

        // Kalman with wheel encoders
        compute_kalman_wheels(&_kal_wheel, time_step_, time_now_s, _meas.left_enc - _meas.prev_left_enc,
                              _meas.right_enc - _meas.prev_right_enc, _pose);

  
/// Printing the log file onto a csv for matlab plotting
if (VERBOSE_PRINT_LOG) {
    controller_print_log(wb_robot_get_time());
}
}
// Close log file Where to put ths 22.05
//(if (fp != NULL)
//fclose(fp);

/**
 * @brief Compute the robot pose using 1 second GPS interval values
 */
 
void controller_get_pose_gps(pose_t _pose_origin) {

    double time_now_s = wb_robot_get_time();

    if (time_now_s - last_gps_time_s > 1.0f) {

        last_gps_time_s = time_now_s;

        _pose.x = _meas.gps[0] - _pose_origin.x;

        _pose.y = -(_meas.gps[2] - _pose_origin.y);

        _pose.heading = -controller_get_heading_gps() + _pose_origin.heading;
        if (VERBOSE_ROBOT_POSE)
            printf("ROBOT pose : %g %g %g\n", _pose.x, _pose.y, RAD2DEG(_pose.heading));
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

    _meas.left_enc = wb_position_sensor_get_value(dev_left_encoder);
    // Store previous value of the right encoder
    _meas.prev_right_enc = _meas.right_enc;

    _meas.right_enc = wb_position_sensor_get_value(dev_right_encoder);

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
                "time; pose_x; pose_y; pose_heading;  gps_x; gps_y; gps_z; right_enc; left_enc; odo_enc_x; odo_enc_y; odo_enc_heading; kal_wheel_x; kal_wheel_y; kal_wheel_heading\n");

    }

    return err;
}


bool controller_init() {
    bool err = false;
    char* robot_name;
    robot_name=(char*) wb_robot_get_name();
    char log_file_name[30];
    sprintf(log_file_name, "log_file_%s.csv", robot_name);
    CATCH(err, controller_init_log(log_file_name));
    return err;
}

/**
 *
 * @brief Printing onto log file variables of interest
 */
void controller_print_log(double time) {

    if (fp != NULL) {
        fprintf(fp, "%g; %g; %g; %g; %g; %g; %g; %g; %g; %g; %g %g\n",
                time, _pose.x, _pose.y, _pose.heading, _meas.right_enc, _meas.left_enc,
                 _odo_enc.x, _odo_enc.y, _odo_enc.heading, _kal_wheel.x,
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


int main(){

return 0;
};


