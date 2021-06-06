/*****************************************************************************/
/* File:         kalman.c                                                    */
/* Version:      1.0                                                         */
/* Date:         06-Jun-21                                                   */
/* Description:  Compute the odometry with a GPS update every second           */
/*                                                                           */
/* Author        Lavnia Schlyter                                             */
/*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "kalman.h"


/*CONSTANTS*/
#define WHEEL_AXIS        0.057        // Distance between the two wheels in meters
#define WHEEL_RADIUS            0.020        // Radius of the wheel in meters
#define K                     0.05           // Estimated 
#define true  1
#define false 0

#define VERBOSE_ACC_KAL false                       // Print the accelerometer Kalman
#define VERBOSE_WHEEL_KAL false                       // Print the wheel encoder Kalman


/// Variables to store the pose of wheel and the position+velocities for accelerometer
static double X_wheel[3][1], X_acc[4][1];
static double X_new_wheel[3][1], X_new_acc[4][1];

/// Variables to store the "Kalman gain"
static double K_wheel[3][3], K_acc[4][2];

/// Store the pose (x,y,\f$\theta\f$)
static pose_t _kal_wheel, _kal_acc;

/// Time step of 16ms
// This is hardcoded in order to not import webots files in here
static const double dt = 0.016;

// Record the last_gps_times
double last_gps_time_whe = 0.0f;
double last_gps_time_acc = 0.0f;

/**
 * Multiply two matrices (mat1 * mat2) of any size and returns a matrix (res)
 * @param m1 Number of rows for mat1
 * @param m2 Number of columns for mat1
 * @param mat1 First matrix size (m1xm2)
 * @param n1 Number of rows for mat2
 * @param n2 Number of columns for mat2
 * @param mat2 Second matrix size (n1xn2)
 * @param res Returns a matrix of size (m1xn2)
 */
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

/**
 * Addition of two matrices
 * @param m1 Number of rows for mat1
 * @param m2 Number of columns for mat1
 * @param mat1 First matrix
 * @param mat2 Second matrix
 * @param res Sum of both
 * @param scale Scaling factor which multiplies every element of mat2 (use 1 if not needed)
 */
void add(int m1, int m2, const double mat1[][m2], const double mat2[][m2], double res[m1][m2], double scale) {
    for (int i = 0; i < m1; i++) {
        for (int j = 0; j < m2; j++) {
            res[i][j] = mat1[i][j] + mat2[i][j] * scale;
        }
    }
}

/**
 * Substraction between two matrices (mat1 - mat2)
 * @param m1 Number of rows for mat1
 * @param m2 Number of columns for mat1
 * @param mat1 First matrix
 * @param mat2 Second matrix
 * @param res Stores the result of substraction
 */
void substract(int m1, int m2, const double mat1[][m2], const double mat2[][m2], double res[m1][m2]) {

    for (int i = 0; i < m1; i++) {

        for (int j = 0; j < m2; j++) {

            res[i][j] = mat1[i][j] - mat2[i][j];
        }
    }

}


/**
 * Computes the determinant of a 3x3 matrix
 * @param mat Matrix of size 3x3
 * @return Determinant of matrix mat
 */
double determinant(double mat[3][3]) {
    double determinant = 0;
    for (int i = 0; i < 3; i++)
        determinant = determinant + (mat[0][i] * (mat[1][(i + 1) % 3] * mat[2][(i + 2) % 3] -
                                                  mat[1][(i + 2) % 3] * mat[2][(i + 1) % 3]));
    return determinant;
}

/**
 * Computes the inverse of a 3x3 matrix
 * @param mat Matrix of size 3x3
 * @param mat_inv Stores the inverse of mat
 */
void inverse(double mat[3][3], double mat_inv[3][3]) {
    double det = determinant(mat);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mat_inv[j][i] = ((mat[(i + 1) % 3][(j + 1) % 3] * mat[(i + 2) % 3][(j + 2) % 3]) -
                             (mat[(i + 1) % 3][(j + 2) % 3] * mat[(i + 2) % 3][(j + 1) % 3])) / det;

        }
    }

}

/**
 * Compute the determinant of a 2x2 matrix
 * @param mat Matrix of size 2x2
 * @return Determinant of 2x2 matrix
 */
double determinant_2(double mat[2][2]) {
    return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
}

/**
 * Computes the inverse of a 2x2 matrix
 * @param mat Matrix of size 2x2
 * @param mat_inv Stores the inverse of mat
 */
void inverse_2(double mat[2][2], double mat_inv[2][2]) {

    double det = determinant_2(mat);

    mat_inv[0][0] = mat[1][1] / det;
    mat_inv[0][1] = -mat[0][1] / det;
    mat_inv[1][0] = -mat[1][0] / det;
    mat_inv[1][1] = mat[0][0] / det;

}

/**
 * Computes the transpose of any matrix and stores result in mat_trans
 * @param row Number of rows in mat
 * @param col Number of columns in mat
 * @param mat Matrix we want to transpose
 * @param mat_trans Transposed matrix
 */
void transpose(int row, int col, double mat[][col], double mat_trans[][row]) {
    int i, j;
    for (i = 0; i < col; i++)
        for (j = 0; j < row; j++)
            mat_trans[i][j] = mat[j][i];
}


/**
 * Computing the position using the wheel encoder and using _pose for updating every second where _pose is the GPS measurement
 * rescaled to the robots original position
 * @param pos_kal_wheel Saves the position computed through kalman
 * @param time_step Time step in webots
 * @param time_now Current time
 * @param meas_ Measurement structure
 * @param Aleft_enc Left wheel encoder
 * @param Aright_enc Reft wheel encoder
 * @param pose_ Pose structure where the true positon are stored
 */
void compute_kalman_wheels(pose_t *pos_kal_wheel, const int time_step, double time_now, double Aleft_enc, double Aright_enc,
                      const pose_t pose_) {
                      
                      

    // Because the GPS is only updated every second, the heading is not very accurate (this was tested in previous versions) and we thus do not update the heading with GPS but keep the odometry value
    double heading =  pos_kal_wheel ->heading;
    double z[3][1] = {{pose_.x}, {pose_.y}, {heading}};    

    // Convert from radians to meters
    Aleft_enc *= WHEEL_RADIUS;
    Aright_enc *= WHEEL_RADIUS;
    
    
	/** Remove abusrd starting values
	
	We noted that the first value of the encoder starts with NAN and causes absurd results, the following check verifies this absurd jump
	And the small mistake in position which occurs because of skiping one iteration is negligable because of the small time step and is caught up with the GPS udpate (in Kalman when in use)
	**/
    if ( (Aleft_enc < 0.3) & (Aright_enc < 0.3) ) {


    const double delta_s = (Aright_enc + Aleft_enc) / 2;
    const double delta_theta = (Aright_enc - Aleft_enc) / (WHEEL_AXIS);

    if (VERBOSE_WHEEL_KAL) {
        printf("===============NEW STEP WHEEL KALMAN==================\n");
        printf("------------------ \n");
        printf("x_wheel = %g,   y_wheel = %g   heading_wheel = %g \n", X_wheel[0][0], X_wheel[1][0], RAD2DEG(X_wheel[2][0]));
    }

    // Actuator noise
    double R[2][2] = {{K * fabs(Aright_enc), 0},
                      {0,                    K * fabs(Aleft_enc)}};

    // Covariance 
    static double Cov[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 0.01}};


    // Pose propagation noise matrix
    double Fx[3][3] = {{1, 0, -delta_s * sin(heading + delta_theta / 2)},
                       {0, 1, delta_s * cos(heading + delta_theta / 2)},
                       {0, 0, 1}};


    double FxT[3][3];
    transpose(3, 3, Fx, FxT);

    double tm1 = 1 / 2 * cos(heading + delta_theta / 2);
    double tm2 = delta_s / (2 * WHEEL_AXIS) * sin(heading + delta_theta / 2);
    double tm3 = delta_s / (2 * WHEEL_AXIS) * cos(heading + delta_theta / 2);
    double tm4 = 1 / (2) * sin(heading + delta_theta / 2);

    // From actuator noise to pose noise incremental
    double Fu[3][2] = {{tm1 - tm2,      tm1 + tm2},
                       {tm4 + tm3,      tm4 - tm3},
                       {1 / WHEEL_AXIS, -1 / WHEEL_AXIS}};

    // Transposed matrix
    double FuT[2][3];
    transpose(3, 2, Fu, FuT);

    /// Start Prediction step
    
    /** Prediction step for kalman with wheel encoders
    	
    	- X_new = X + pred
    	- Cov_new = Fx*Cov*Fx^T + Fu*R*Fu^T
    
    **/

    double pred[3][1] = {{delta_s * cos(heading + delta_theta / 2)},
                      {delta_s * sin(heading + delta_theta / 2)},
                      {delta_theta}};

    // X(t+1) = X(t) + pred
    add(3, 1, X_wheel, pred, X_new_wheel, 1);

    // Update noise covariance matrix
    // Cov = Fx * Cov * Fx^T + Fu*R*Fu^T
    double tmp1[3][3];
    // tmp1 = Fx *Cov
    multiply(3, 3, Fx, 3, 3, Cov, tmp1);
    double tmp2[3][3];
    // tmp2 = Fx *Cov *FxT
    multiply(3, 3, tmp1, 3, 3, FxT, tmp2);

    double tmp3[3][2];
    // tmp3 = Fu*R
    multiply(3, 2, Fu, 2, 2, R, tmp3);
    double tmp4[3][3];
    // tmp4 = Fu*R*FuT
    multiply(3, 2, tmp3, 2, 3, FuT, tmp4);

    // Updated noise matrix
    // Cov = Fx * Cov * FxT + Fu * R * FuT
    add(3, 3, tmp2, tmp4, Cov, 1);

    /// End of prediction step 

    /// Correction step
    
    /** Updating step for kalman with wheel encoders
    	
    	- K = Cov*C^T*(C)*Cov*C^T + Q)^-1 // Kalman Gain
    	- X_new = X + K*(z - C*X)         // Updating measure
    	- Cov_new = (I - K*C)*Cov         // Update covariance
    	
    
    **/
    
    // Measurement model 
    static double C[3][3] = {{1, 0, 0},
                             {0, 1, 0},
                             {0, 0, 1}};

    static double CT[3][3];
    transpose(3, 3, C, CT);

    // Covariance matrix, measurement noise (GPS)
    // Do not trust the heading of GPS 
    static double Q[3][3] = {{0.001, 0,     0},
                             {0,     0.001, 0},
                             {0,     0,     1}};

    // Identity matrix
    static double Id3[3][3] = {{1, 0, 0},
                               {0, 1, 0},
                               {0, 0, 1}};

    // Update using pose/GPS at every second
    
    if (time_now - last_gps_time_whe > 1.0f) {
   
    
        last_gps_time_whe = time_now;

        /// Start Compute Kalman gain K
        
        double tmp5[3][3];
        /// tmp5 = Cov_new * CT
        multiply(3, 3, Cov, 3, 3, CT, tmp5);
        double tmp6[3][3];
        /// tmp6 =   C * Cov_new * CT
        multiply(3, 3, C, 3, 3, tmp5, tmp6);

        /// tmp6 = tmp6 + Q  = C*Cov_new*CT + Q
        add(3, 3, tmp6, Q, tmp6, 1);

        double tmp7[3][3];
        /// inverse = inv(C*Cov_new*C_trans + Q)
        inverse(tmp6, tmp7);

        /// K = Cov_new * CT * inv(C*Cov_new*C_trans + Q)
        multiply(3, 3, tmp5, 3, 3, tmp7, K_wheel);

	/// End of Kalman gain computation


        /// Start of updating computation // new X when "true" pose (gps) available
        
        double tmp8[3][1];
        /// tmp8 = C*X_new
        multiply(3, 3, C, 3, 1, X_new_wheel, tmp8);
        /// tmp8 = z - C*X_new = z - tmp3
        substract(3, 1, z, tmp8, tmp8);
        double tmp9[3][1];
        /// tmp9 = K*tmp8 = K*(z - C*X_new)
        multiply(3, 3, K_wheel, 3, 1, tmp8, tmp9);
        /// X_new = X_new + K*(z - C*X_new)
        add(3, 1, tmp9, X_new_wheel, X_new_wheel, 1);
        
        /// End of updating computation
        

        /// Start Compute new covariance matrix
        
        double tmp10[3][3];
        /// tmp10 = K*C
        multiply(3, 3, K_wheel, 3, 3, C, tmp10);
        double tmp11[3][3];
        /// tmp11 = eye(4) - K*C
        substract(3, 3, Id3, tmp10, tmp11);
        double tmp12[3][3];
        /// Cov_new = Cov_new*(eye(4) - K*C)
        multiply(3, 3, tmp11, 3, 3, Cov, tmp12);
        
        /// End Compute new covariance matrix
        

        // Assign to Cov the new values stored in tmp12

        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                Cov[i][j] = tmp12[i][j];
    }
    
      	// Keep orientation within 0, 2pi
  	while (X_new_wheel[2][0] > 2*M_PI) {
          	X_new_wheel[2][0] -= 2.0*M_PI;
  	
  	}
  	
  	while (X_new_wheel[2][0] < 0) {
          	X_new_wheel[2][0] += 2.0*M_PI;
  	
  	}
    
    


    /// Storing computed position into pose vector
    _kal_wheel.x = X_new_wheel[0][0];
    _kal_wheel.y = X_new_wheel[1][0];
    _kal_wheel.heading = X_new_wheel[2][0];

    X_wheel[0][0] = X_new_wheel[0][0];
    X_wheel[1][0] = X_new_wheel[1][0];
    X_wheel[2][0] = X_new_wheel[2][0];

    memcpy(pos_kal_wheel, &_kal_wheel, sizeof(pose_t));
    }


}

/**
 * Computing the position using the accelerometer and using _pose for updating every second where _pose is the GPS measurement
 * rescaled to the robots original position, the heading is given by the wheel encoder
 * @param pos_kal_acc Saves the position computed through kalman
 * @param time_step Time step in webots
 * @param time_now Current time
 * @param heading Wheel encoder heading
 * @param meas_ Measurement structure
 * @param pose_ Pose structure where the true positon are stored
 */

void compute_kalman_acc(pose_t *pos_kal_acc, const int time_step, double time_now, const double heading,
                        const measurement_t meas_, const pose_t pose_) {
                        
    
    /** Acceleration
    Because the accelerometer stands on the robot frame and we consider the robot to not slide, we only account for its "forward" acceleration (2nd component in acceleration vector); this is then projected onto the revelant frame 
    **/    
    
    // Remove bias          
    double acc_r = ( meas_.acc[1] - meas_.acc_mean[1]);
    
    // Project
    double acceleration[2][1] = {{acc_r*cos(heading)}, {acc_r*sin(heading)}};

    // True position vector (updated using GPS) in robot frame
    double z[2][1] = {{pose_.x}, {pose_.y}};


    if (VERBOSE_ACC_KAL) {
        printf("===============NEW STEP ACCELEROMETER KALMAN==================\n");
        printf("GPSx = %g,   GPSy = %g\n", meas_.gps[0], meas_.gps[2]);
        printf("------------------ \n");
        printf("x_acc = %g,   y_acc = %g  \n", X_acc[0][0], X_acc[1][0]);
        printf("-----------\n");
        printf("Heading acc = %g \n", heading);
    }


    /** Covariance representing motion noise
    	Because we integrate twice for the position we estimate the noise to be a litte higher on the  (x,y)
    	Reason for the slightly higher values
    **/
    static double R[4][4] = {
            {0.05, 0,    0,    0},
            {0,    0.05, 0,    0},
            {0,    0,    0.01, 0},
            {0,    0,    0,    0.01}};

    /// State uncertainty
    static double Cov[4][4] = {
            {0.001, 0,     0,     0},
            {0,     0.001, 0,     0},
            {0,     0,     0.001, 0},
            {0,     0,     0,     0.001}};

    /// Measurement model
    static double C[2][4] = {{1, 0, 0, 0},
                             {0, 1, 0, 0}};

    /// Covariance matrix, measurement noise
    static double Q[2][2] = {{0.001,  0.},
                             {0., 0.001}};

    /// Transposed of C matrix
    static double CT[4][2];
    transpose(2, 4, C, CT);

    // Identity matrix
    static double Id4[4][4] = {{1, 0, 0, 0},
                               {0, 1, 0, 0},
                               {0, 0, 1, 0},
                               {0, 0, 0, 1}};

    /// State transition
    static double A[4][4] = {
            {1, 0, dt, 0},
            {0, 1, 0,  dt},
            {0, 0, 1,  0},
            {0, 0, 0,  1}};

    static double AT[4][4];
    transpose(4, 4, A, AT);

    /// Input Control matrix
    static double B[4][2] = {
            {0,  0},
            {0,  0},
            {dt, 0},
            {0,  dt}};

    /** Prediction step for kalman with accelerometer
    	
    	- X_new = A*X + B*acc
    	- A*Cov*A^T + R*dt
    
    **/

    double tmp1[4][1];
    // tmp1 = A*X
    multiply(4, 4, A, 4, 1, X_acc, tmp1);

    double tmp2[4][1];
    // tmp2 = B*acc
    multiply(4, 2, B, 2, 1, acceleration, tmp2);

    // X_new = A*X + B*acc
    add(4, 1, tmp1, tmp2, X_new_acc, 1);

    /// Compute Cov_new without true pose (gps)
    
    double tmp4[4][4];
    //tmp4 = A*Cov
    multiply(4, 4, A, 4, 4, Cov, tmp4);
    double tmp5[4][4];
    // tmp5 = A*Cov*A^T
    multiply(4, 4, tmp4, 4, 4, AT, tmp5);

    // The dt as scaling factor is drawn from the lab on Kalman on webots
    // Cov = A*Cov*A^T + R*dt
    add(4, 4, tmp5, R, Cov, dt);

     /** Updating step for kalman with accelerometer
    	
    	- K = Cov*C^T*(C)*Cov*C^T + Q)^-1 // Kalman Gain
    	- X_new = X + K*(z - C*X)         // Updating measure
    	- Cov_new = (I - K*C)*Cov         // Update covariance
    	
    
    **/
    
    if (time_now - last_gps_time_acc > 1.0f) {

        last_gps_time_acc = time_now;

        /// Start Compute Kalman gain
        
        double tmp8[4][2];
        // tmp8 = Cov_new*CT
        multiply(4, 4, Cov, 4, 2, CT, tmp8);

        double tmp9[2][2];
        // tmp9 = C*Cov_new*CT
        multiply(2, 4, C, 4, 2, tmp8, tmp9);
        // tmp9 = C*Cov_new*CT + Q
        add(2, 2, tmp9, Q, tmp9, 1);

        double tmp10[2][2];
        // inverse = inv(C*Cov_new*CT)
        inverse_2(tmp9, tmp10);

        // K = Cov_new*CT* inv(C* Cov_new*CT + Q)
        multiply(4, 2, tmp8, 2, 2, tmp10, K_acc);
        
        /// End of Kalman gain computation
        

	/// Start Updating measure
	
        double tmp3[2][1];
        // tmp3 = C*X_new (2x1)
        multiply(2, 4, C, 4, 1, X_new_acc, tmp3);

        double tmp11[4][1];
        // tmp3 = z - C*X_new = z - tmp3
        substract(2, 1, z, tmp3, tmp3);
        // tmp3 = K*tmp3 = K*(z - C*X_new)
        multiply(4, 2, K_acc, 2, 1, tmp3, tmp11);
        // X_new = X_new + K*(z - C*X_new)
        add(4, 1, tmp11, X_new_acc, X_new_acc, 1);
        
        /// End of Updating measure
        
        
        /// Start Compute new covariance matrix
        
        double tmp6[4][4];
        // tmp6 = K*C
        multiply(4, 2, K_acc, 2, 4, C, tmp6);

        double tmp7[4][4];
        // tmp7 = eye(4) - K*C
        substract(4, 4, Id4, tmp6, tmp7);
        // Cov_new = Cov_new*(eye(4) - K*C)
        multiply(4, 4, Cov, 4, 4, tmp7, tmp6);
        
        /// End updating of covariance
        
        // Assign to Cov the new values stored in tmp6
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                Cov[i][j] = tmp6[i][j];
        
    }

    /// Storing computed position into "kalman pose" vector
    _kal_acc.x = X_new_acc[0][0];
    _kal_acc.y = X_new_acc[1][0];
    _kal_acc.heading = heading;


    X_acc[0][0] = X_new_acc[0][0];
    X_acc[1][0] = X_new_acc[1][0];
    X_acc[2][0] = X_new_acc[2][0];
    X_acc[3][0] = X_new_acc[3][0];

    memcpy(pos_kal_acc, &_kal_acc, sizeof(pose_t));


}

// Reset the values to zero 
void kal_reset()
{
 	memset(&X_wheel, 0 , sizeof(double));

	memset(&X_acc, 0 , sizeof(double));

	memset(&X_new_wheel, 0 , sizeof(double));
	
        memset(&X_new_acc, 0 , sizeof(double));

	memset(&_kal_wheel, 0 , sizeof(pose_t));

	memset(&_kal_acc, 0 , sizeof(pose_t));
}
