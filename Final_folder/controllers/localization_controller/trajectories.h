#ifndef TRAJECTORIES_H
#define TRAJECTORIES_H 

#include <webots/robot.h>
#include <webots/motor.h>

// ## DO NOT MODIFY THIS
void trajectory_1(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor);
void trajectory_2(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor);

// This trajectory is used for calibration. It computes the bias of the accelerometer by leaving the robot at a stand-still
void trajectory_3(WbDeviceTag dev_left_motor, WbDeviceTag dev_right_motor);

#endif