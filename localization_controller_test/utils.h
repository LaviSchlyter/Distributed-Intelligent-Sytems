#ifndef UTILS_H
#define UTILS_H 


#define TRUE 1
#define FALSE 0
#define RAD2DEG(X)      X / M_PI * 180.0

typedef struct 
{
  double x;
  double y;
  double heading;
} pose_t;

typedef struct 
{
  double prev_gps[3];
  double gps[3];
  double acc_mean[3];
  double acc[3];
  double prev_left_enc;
  double left_enc;
  double prev_right_enc;
  double right_enc;
  double acc_mean_calibration[3]; // added for calibrating 
} measurement_t;


#endif




