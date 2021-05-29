#ifndef LOCALIZATION_H
#define LOCALIZATION_H 
#include "utils.h"
#include "odometry.h"
#include "kalman.h"


#include <webots/robot.h>
#include <webots/motor.h>

static pose_t _pose, _odo_enc, _kal_wheel;
static measurement_t _meas; // See class in util


void init_devices(int ts);

static void controller_get_pose_gps();

static void controller_get_gps();

static double controller_get_heading_gps();

static void controller_get_encoder(); 

void compute_localization(pose_t* _pose_origin);

static bool controller_init_log(const char *filename);

static bool controller_init();

static void controller_print_log(double time);
static bool controller_error(bool test, const char *message, int line, const char *fileName);


#endif
