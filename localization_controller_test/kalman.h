#ifndef KALMAN_H
#define KALMAN_H

#include "utils.h"

/// Documentation in c file
void compute_kalman_acc(pose_t* pos_kal_acc, const int time_step, double time_now, const double heading, const measurement_t meas_, const pose_t pose_);
void compute_kalman_wheels(pose_t* pos_kal_wheel, const int time_step, double time_now, double Aleft_enc, double Aright_enc,const pose_t pose_);
#endif