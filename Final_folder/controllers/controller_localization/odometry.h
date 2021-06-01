#ifndef ODOMETRY_H
#define ODOMETRY_H 

#include "utils.h"

void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3], const double heading);
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc);
void odo_reset(int time_step);

#endif
