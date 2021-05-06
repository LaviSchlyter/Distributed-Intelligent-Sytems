#ifndef KALMAN_H
#define KALMAN_H

#include "utils.h"

typedef struct 
{
  double x;
  double y;
  double vx;
  double vy;
  double heading;
} kalman_t;


void compute_kalman_acc(pose_t* pos_kal_acc, const int time_step, double time_now, const pose_t GPS, const double heading, const measurement_t meas_);
void compute_kalman_wheels(pose_t* pos_kal_wheel, const int time_step, double time_now, const pose_t GPS, const double heading, double Aleft_enc, double Aright_enc);





#endif