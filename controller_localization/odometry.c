#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>

#include <odometry.h>

// Code with minor changes taken from Lab03 DIS-EPFL
//-----------------------------------------------------------------------------------//
/*CONSTANTES*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	        0.0205		// Radius of the wheel in meter

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC_BONUS false     	        // Print odometry values computed with wheel encoders (Bonus)
#define VERBOSE_ODO_ENC false     			// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC true    			// Print odometry values computed with accelerometer
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc, _odo_pose_enc_bonus;
//-----------------------------------------------------------------------------------//

/**
 * @brief      Compute the odometry using the acceleration
 *
 * @param      odo       The odometry
 * @param[in]  acc       The acceleration
 * @param[in]  acc_mean  The acc mean
 */
void odo_compute_acc(pose_t* odo, const double acc[3], const double acc_mean[3], const double heading)
{
        // Compute acceleration in frame A + remove the bias (in 2D motion)
	double acc_wx = ( acc[1] - acc_mean[1])*cos(heading);
	double acc_wy = -( acc[0] - acc_mean[0])*sin(heading);
	//printf("Acceleration : %f %f \n", acc_wx, acc_wy);
	
	
        // 2D motion model //
        

        // Speed
	_odo_speed_acc.x += acc_wx *_T;
	_odo_speed_acc.y += acc_wy *_T;

        // Position
	_odo_pose_acc.x += _odo_speed_acc.x * _T*cos(heading);
	_odo_pose_acc.y += _odo_speed_acc.y * _T*sin(heading);

        // Upgrade heading with wheel encoder value 
        _odo_pose_acc.heading = heading;

	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
	

        
 	//printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));


 	
}

/**
 * @brief      Compute the odometry using the wheel encoders 
 *
 * @param      odo         The odometry
 * @param[in]  Aleft_enc   The delta left encoder
 * @param[in]  Aright_enc  The delta right encoder
 */
void odo_compute_encoders(pose_t* odo, double Aleft_enc, double Aright_enc)
{
	// Convert wheel encoders units (radians) into meters
	Aleft_enc  *= WHEEL_RADIUS;

	Aright_enc *= WHEEL_RADIUS;

	// Compute forward speed and angular speed
	double omega = ( Aright_enc - Aleft_enc ) / ( WHEEL_AXIS * _T );

	double speed = ( Aright_enc + Aleft_enc ) / ( 2.0 * _T );

	// Apply rotation (Body to World)

        double a =_odo_pose_enc.heading;

	double speed_wx = speed * cos(a);

	double speed_wy = speed * sin(a);
	

	// Integration : Euler method
	_odo_pose_enc.x += speed_wx * _T;

	_odo_pose_enc.y += speed_wy * _T;

	_odo_pose_enc.heading += omega * _T;

	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

    	//printf("ODO with wheel encoders : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading) );
    	
}

/**
 * @brief      Reset the odometry to zeros
 *
 * @param[in]  time_step  The time step used in the simulation in miliseconds
 */
void odo_reset(int time_step)
{

 	memset(&_odo_pose_acc, 0 , sizeof(pose_t));

	memset(&_odo_speed_acc, 0 , sizeof(pose_t));

	memset(&_odo_pose_enc, 0 , sizeof(pose_t));

	memset(&_odo_pose_enc_bonus, 0 , sizeof(pose_t));

	_T = time_step / 1000.0;
}



