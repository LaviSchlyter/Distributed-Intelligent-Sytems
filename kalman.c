#include <stdio.h>
#include <string.h>
#include <math.h>
#include <odometry.h>
#include "kalman.h"


/** Kalman filter
  * Framework to estimate an unknown variable using a series of measurements containing noise.
  * Best possible linear estimator
  *
*/
/*CONSTANTES*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	        0.0205		// Radius of the wheel in meter
#define K                     0.01                               // da,mn

#define true  1
#define false 0

#define VERBOSE_GPS_pose true                  // Print GPS pose
#define VERBOSE_X true                         // Print X
#define VERBOSE_X_new false                    // Print X_new
#define VERBOSE_WHEELS true                    // Print delta_s and delta_theta
#define VERBOSE_Whead true                         // Print Heading wheel
#define VERBOSE_NEW true                        // Print new line step
#define VERBOSE_ACC false                       // Print the accelerometer Kalman
#define VERBOSE_WHEEL true                       // Print the wheel encoder Kalman
int time_step;
static kalman_t kal_pos, kal_pos_wheel, kal_pos_acc;
static double X_wheel[3], X_acc[4];
static double K_wheel[3][3], K_acc[4][2];
static double X_new_wheel[3], X_new_acc[4];
static pose_t _kal_wheel,_kal_acc;


double last_gps_time_ = 0.0f;

// Multiplies two matrices mat1[][] and mat2[][] // and returns result. // (m1) x (m2) and (n1) x (n2) are dimensions // of given matrices.
void multiply(int m1, int m2, const double mat1[][m2], int n1, int n2, const double mat2[][n2], double res[m1][n2]) {


    for (int i = 0; i < m1; i++) {
        for (int j = 0; j < n2; j++) {
            res[i][j] = 0;
            for (int k = 0; k < m2; k++) {
                *(*(res + i) + j) += *(*(mat1 + i) + k) * *(*(mat2 + k) + j);
            }
        }
    }
}

void add(int m1, int m2, const double mat1[][m2], const double mat2[][m2], double res[m1][m2], double scale) {


    for (int i = 0; i < m1; i++) {
        for (int j = 0; j < m2; j++) {
            //res[i][j] = 0;
            res[i][j] = mat1[i][j] + mat2[i][j] * scale;
        }
    }

}

void substract(int m1, int m2, const double mat1[][m2], const double mat2[][m2], double res[m1][m2]) {


    for (int i = 0; i < m1; i++) {
        for (int j = 0; j < m2; j++) {

            res[i][j] = mat1[i][j] - mat2[i][j];
        }
    }

}


double determinant(double mat[3][3]) {
    double determinant = 0;
    for(int i = 0; i< 3; i++)
        determinant = determinant + (mat[0][i]*(mat[1][(i+1)%3]*mat[2][(i+2)%3] - mat[1][(i+2)%3]*mat[2][(i+1)%3]));
    return determinant;
}

void inverse(double mat[3][3], double mat_inv[3][3]) {
    double det = determinant(mat);
      for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3 ; j++) {
           mat_inv[j][i] = ((mat[(i+1)%3][(j+1)%3] * mat[(i+2)%3][(j+2)%3]) - (mat[(i+1)%3][(j+2)%3]*mat[(i+2)%3][(j+1)%3]))/ det;

     }
   }

    


}


void transpose(int row, int col, double mat[][col], double mat_trans[][row]) {
  int i, j;
    for (i = 0; i < col; i++)
        for (j = 0; j < row; j++)
            mat_trans[i][j] = mat[j][i];


}


void compute_kalman_wheels(pose_t* pos_kal_wheel, const int time_step, double time_now, const pose_t GPS, const double heading, double Aleft_enc, double Aright_enc) {
    
    if (VERBOSE_NEW)
      printf("===============NEW STEP WHEEL KALMAN==================\n");
      
    // True GPS pose 
    double z[3] = {GPS.x, GPS.y, GPS.heading};
    // Convert from radians to meters
    Aleft_enc  *= WHEEL_RADIUS;

    Aright_enc *= WHEEL_RADIUS;
    
    
    const double delta_s = (Aright_enc + Aleft_enc) / 2;
    const double delta_theta =  (Aright_enc - Aleft_enc)/(WHEEL_AXIS);
    if (VERBOSE_WHEEL) {
      if (VERBOSE_GPS_pose) {
        printf("GPSx = %g,   GPSy = %g, GPShead = %g \n", GPS.x, GPS.y, RAD2DEG(GPS.heading));
      }
      
      if (VERBOSE_X) {
        printf("------------------ \n");
        printf("X = %g,   Y = %g   HEAD = %g \n", X_wheel[0],X_wheel[1], RAD2DEG(X_wheel[2]));
      }
    
      if (VERBOSE_X_new) {  
        printf("------------------ \n");
        printf("X_newx = %g,   X_newy = %g   X_newhead = %g \n", X_new_wheel[0],X_new_wheel[1], RAD2DEG(X_new_wheel[2]));
        }
      
      if (VERBOSE_Whead) {
        printf("-----------\n");
        printf("Heading wheel = %g \n", heading);
        
      }
      
      
      
      if (VERBOSE_WHEELS) {
      
        printf("-----------------------\n");
        printf("Delta_s = %g \n", delta_s);
        printf("-----------------------\n");
        printf("Delta_theta in degrees= %g\n", RAD2DEG(delta_theta));
      
      }
    }
 
    static const double dt = 0.016; // time step of 16ms

    // Actuator noise
    double R[2][2] = {{K*fabs(Aright_enc), 0},
                             {0, K*fabs(Aleft_enc)}};

    // Covariance 
     
    static double Cov[3][3] = {
            {0.01, 0,     0},
            {0,     0.01, 0},
            {0,     0,     0.01}};

   
   // Pose propagation noise matrix 
   double Fx[3][3] = {{1, 0, -delta_s*sin(heading + delta_theta/2)},
                      {0, 1, delta_s*cos(heading + delta_theta/2)},
                      {0, 0, 1}};
  double tm1 = 1/2*cos(heading + delta_theta/2);
  double tm2 = delta_s/(2*WHEEL_AXIS)*sin(heading + delta_theta/2);
  double tm3 = delta_s/(2*WHEEL_AXIS)*cos(heading + delta_theta/2);
  double tm4 = 1/(2)*sin(heading + delta_theta/2);
  
  double FxT[3][3];
  transpose(3, 3, Fx, FxT);

   
   // From actuator noise to pose noise incremental       
   double Fu[3][2] = {{tm1 - tm2, tm1 + tm2},
                       {tm4 + tm3, tm4 - tm3},
                       {1/WHEEL_AXIS, -1/WHEEL_AXIS}}; 
   
   // Transposed matrix 
   double FuT[2][3];
   

   transpose(3,2, Fu, FuT);
   // Prediction step 
   
    // mu(t+1) = m(t) + pred
    double pred[3][1] = {{delta_s*cos(heading + delta_theta/2)}, 
                        {delta_s*sin(heading + delta_theta/2)},
                         {delta_theta}};
    add(3 , 1, X_wheel, pred, X_new_wheel, 1);
    
    // Update noise covariance matrix
    // Sigma = Fx * Sigma * Fx^T + Fu*R*Fu^T
    double tmp1[3][3], tmp2[3][3], tmp3[3][2], tmp4[3][3];
    
    
    multiply(3, 3, Fx, 3, 3, Cov, tmp1); // tmp1 = Fx *Cov

    


    multiply(3, 3, tmp1, 3, 3, FxT, tmp2); // tmp2 = tmp1 *FxT
    multiply(3, 2, Fu, 2, 2, R, tmp3); // tmp3 = Fu*R


    
    multiply(3, 2, tmp3, 2, 3, FuT, tmp4); // tmp4 = Fu*R*FuT

    // Updated noise matrix 
    add(3, 3, tmp2, tmp4, Cov, 1);


    
    // End of prediction step 
    
    // Correction step
 

    // Measurement model 
    static double C[3][3] = {{1, 0, 0},
                             {0, 1, 0},
                             {0, 0, 1}};
                             
   static double CT[3][3];
   transpose(3, 3, C, CT);
                                
    
    // Covariance matrix, measurement noise
    static double Q[3][3] = {{0.1, 0, 0},
                             {0, 0.1, 0},
                             {0, 0, 0.1}};
                             


    // Identity matrix
    static double Id3[3][3] = {{1, 0, 0},
                               {0, 1, 0},
                               {0, 0, 1}};
    
    double tmp5[3][3], tmp6[3][3], tmp7[3][3];
    double tmp8[3][1], tmp9[3][3], tmp10[3][3], tmp11[3][3];
    
  
    // Every second update using GPS pose 
    
    //if (time_now - last_gps_time_ > 1.0f) {
    if (time_now - last_gps_time_ > 0.5f) {
      printf("Entered kalman:\n");
    
      last_gps_time_ = time_now;

      // Compute K
      multiply(3, 3, Cov, 3, 3, CT, tmp5); // D = Cov_new*C_trans
      multiply(3, 3, C, 3, 3, tmp5, tmp6); // Y = C* D = C*Cov_new*C_trans
      
      add(3, 3, tmp6, Q, tmp6, 1); // Y = Y + Q = C* D = C*Cov_new*C_trans + Q
      
      inverse(tmp6, tmp7); // inverse = inv(Y) = inv(C*Cov_new*C_trans + Q)
      
      multiply(3, 3, tmp5, 3, 3, tmp7, K_wheel); // K = D* inv(C* D + Q)
      
  
      // Compute new X when gps available
      multiply(3, 3, C, 3, 1, X_new_wheel, tmp8); // tmp8 = C*X_new
      substract(3, 1, z, tmp8, tmp8); // tmp8 = z - C*X_new = z - tmp3
      
      multiply(3, 3, K_wheel, 3, 1, tmp8, tmp8); // tmp8 = K*tmp8 = K*(z - C*X_new)
      add(3, 1, tmp8, X_new_wheel, X_new_wheel, 1); // X_new = X_new + K*(z - C*X_new)
  
      // Compute new covariance matrix
      multiply(3, 3, K_wheel, 3, 3, C, tmp9); // tmp9 = K*C
      substract(3, 3, Id3, tmp9, tmp10); // tmp10 = eye(4) - K*C
      multiply(3, 3, tmp10, 3, 3, Cov, tmp11); // Cov_new = Cov_new*(eye(4) - K*C)
      // Assign to Cov the new values stored in tmp6
      for (int i = 0; i < 3; i++) 
          for (int j = 0; j < 3; j++)
              Cov[i][j] = tmp11[i][j];
      }

      _kal_wheel.x = X_new_wheel[0];
      _kal_wheel.y = X_new_wheel[1];
      _kal_wheel.heading = X_new_wheel[2];

      X_wheel[0] = X_new_wheel[0];
      X_wheel[1] = X_new_wheel[1];
      X_wheel[2] = X_new_wheel[2];

      memcpy(pos_kal_wheel, &_kal_wheel, sizeof(pose_t));
      

}




void compute_kalman_acc(pose_t* pos_kal_acc, const int time_step, double time_now, const pose_t GPS, const double heading, const measurement_t meas_) {


    double acc_wx = ( meas_.acc[1] - meas_.acc_mean[1]);
    double acc_wy = ( meas_.acc[0] - meas_.acc_mean[0]);
    double acceleration[2] = {acc_wx, -acc_wy};
    
    if (VERBOSE_ACC) {
    
      if (VERBOSE_NEW)
        printf("===============NEW STEP ACCELEROMETER KALMAN==================\n");
      
      
      if (VERBOSE_GPS_pose) {
        printf("GPSx = %g,   GPSy = %g, GPShead = %g \n", GPS.x, GPS.y, RAD2DEG(GPS.heading));
      }
      
      if (VERBOSE_X) {
        printf("------------------ \n");
        printf("X = %g,   Y = %g  \n", X_acc[0],X_acc[1]);
      }
    
      if (VERBOSE_X_new) {  
        printf("------------------ \n");
        printf("X_newx = %g,   X_newy = %g\n", X_new_acc[0],X_new_acc[1]);
        }
      
      if (VERBOSE_Whead) {
        printf("-----------\n");
        printf("Heading wheel = %g \n", heading);
        
      }
    }
    
    // True GPS pose 
    double z[2] = {GPS.x, GPS.y};
    
 
    static const double dt = 0.016; // time step of 16ms
    
        // Covariance representing motion noise //quite robust with Kalman
    static double R[4][4] = {
            {0.05, 0,    0,    0},
            {0,    005, 0,    0},
            {0,    0,    0.01, 0},
            {0,    0,    0,    0.01}};

    // State uncertainty // trst and trial, with true and prediction
    static double Cov[4][4] = {
            {0.001, 0,     0,     0},
            {0,     0.001, 0,     0},
            {0,     0,     0.001, 0},
            {0,     0,     0,     0.001}};



    // Measurement model 
    static double C[2][4] = {{1, 0, 0, 0},
                             {0, 1, 0, 0}};
    
    // Covariance matrix, measurement noise
    static double Q[2][2] = {{1., 0.},
                             {0., 1.}};

    // Transposed of C matrix 
    static double CT[4][2];
    transpose(2, 4, C, CT);
     
 // Identity matrix
    static double Id4[4][4] = {{1, 0, 0, 0},
                               {0, 1, 0, 0},
                               {0, 0, 1, 0},
                               {0, 0, 0, 1}};

    // Motion/Process model
    static double A[4][4] = {
            {1, 0, dt, 0},
            {0, 1, 0,  dt},
            {0, 0, 1,  0},
            {0, 0, 0,  1}};

    // Doesn't change
    static double AT[4][4];
    transpose(4, 4, A, AT);
    


    // Control matrix
    static double B[4][2] = {
            {0,  0},
            {0,  0},
            {dt, 0},
            {0,  dt}};

    
  
    // Every second update using GPS pose 
    
    double tmp1[4], tmp2[4], tmp3[2];
    double tmp4[4][4], tmp5[4][4], tmp6[4][4], tmp7[4][4];
    double tmp8[4][2], tmp9[2][2], tmp10[2][2];

    // Compute X_new wihtout GPS 

    
    multiply(4, 4, A, 4, 1, X_acc, tmp1);


    multiply(4, 2, B, 2, 1, acceleration, tmp2);

    add(4, 1, tmp1, tmp2, X_new_acc, 1);


    // Compute Cov_new without gps

    multiply(4, 4, A, 4, 4, Cov, tmp4);
    multiply(4, 4, tmp4, 4, 4, AT, tmp5);
    add(4, 4, tmp5, R, Cov, dt);
    
    
   if (time_now - last_gps_time_ > 1.0f) {
      printf("Entered kalman:\n");
    
      last_gps_time_ = time_now;

      // Compute K
      multiply(4, 4, Cov, 4, 2, CT, tmp8); // D = Cov_new*C_trans
      multiply(2, 4, C, 4, 2, tmp8, tmp9); // Y = C* D = C*Cov_new*C_trans
      add(4, 4, tmp9, Q, tmp9, 1); // Y = Y + Q = C* D = C*Cov_new*C_trans + Q
      inverse(tmp9, tmp10); // inverse = inv(Y) = inv(C*Cov_new*C_trans)
      multiply(4, 2, tmp8, 2, 2, tmp10, K_acc); // K = D* inv(C* D + Q)
  
  
      // Compute new X when gps available
      multiply(2, 4, C, 4, 1, X_new_acc, tmp3); // tmp3 = C*X_new
      substract(4, 1, z, tmp3, tmp3); // tmp3 = z - C*X_new = z - tmp3
      multiply(4, 2, K_acc, 2, 1, tmp3, tmp3); // tmp3 = K*tmp3 = K*(z - C*X_new)
      add(4, 1, tmp3, X_new_acc, X_new_acc, 1); // X_new = X_new + K*(z - C*X_new)
  
      // Compute new covariance matrix
      multiply(4, 2, K_acc, 2, 4, C, tmp6); // tmp6 = K*C
      substract(4, 4, Id4, tmp6, tmp7); // tmp7 = eye(4) - K*C
      multiply(4, 4, Cov, 4, 4, tmp7, tmp6); // Cov_new = Cov_new*(eye(4) - K*C)
      // Assign to Cov the new values stored in tmp6
      for (int i = 0; i < 4; i++) 
          for (int j = 0; j < 4; j++)
              Cov[i][j] = tmp6[i][j];
      }

  
      _kal_acc.x = X_new_acc[0];
      _kal_acc.y = X_new_acc[1];
      _kal_acc.heading = heading;
      
      
      X_acc[0] = X_new_acc[0];
      X_acc[1] = X_new_acc[1];
      X_acc[2] = X_new_acc[2];
      X_acc[3] = X_new_acc[3];

      memcpy(pos_kal_acc, &_kal_acc, sizeof(pose_t));
      

}

