/**************************************************************************************************************************/
/* File:         odometry.c                                         */
/* Version:      1.0                                                               */
/* Date:         06-Jun-21                                                         */
/* Description:  Computation for locating with accelerometer and wheel encoders   */
/*                                                                                 */
/**************************************************************************************************************************/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include "odometry.h"

// Code with minor changes taken from Lab03 DIS-EPFL
//-----------------------------------------------------------------------------------//
/*CONSTANTS*/
#define WHEEL_AXIS 		0.057 		// Distance between the two wheels in meter
#define WHEEL_RADIUS 	        0.0205		// Radius of the wheel in meter

/*VERBOSE_FLAGS*/
#define VERBOSE_ODO_ENC false     			// Print odometry values computed with wheel encoders
#define VERBOSE_ODO_ACC false    			// Print odometry values computed with accelerometer
//-----------------------------------------------------------------------------------//
/*GLOBAL*/
static double _T;

static pose_t _odo_pose_acc, _odo_speed_acc, _odo_pose_enc;
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
        /// Compute acceleration + remove the bias 
        
	double acc_wx = ( acc[1] - acc_mean[1])*cos(heading);
	double acc_wy = -( acc[0] - acc_mean[0])*sin(heading);
	

        // Speed
	_odo_speed_acc.x += acc_wx *_T;
	_odo_speed_acc.y += acc_wy *_T;

        // Position
	_odo_pose_acc.x += _odo_speed_acc.x * _T*cos(heading);
	_odo_pose_acc.y += _odo_speed_acc.y * _T*sin(heading);

        // Upgrade heading with wheel encoder value 
        _odo_pose_acc.heading = heading;

	memcpy(odo, &_odo_pose_acc, sizeof(pose_t));
	
        if (VERBOSE_ODO_ACC)
              printf("ODO with acceleration : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading));


 	
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
	

	/** Remove abusrd starting values
	
	We noted that the first value of the encoder starts with NAN and causes absurd results, the following check verifies this absurd jump
	And the small mistake in position which occurs because of skiping one iteration is negligable because of the small time step and is caught up with the GPS udpate (in Kalman when in use)
	**/
	if (Aleft_enc <0.30 & Aright_enc < 0.3) {
	

		// Integration : Euler method
		_odo_pose_enc.x += speed_wx * _T;

		_odo_pose_enc.y += speed_wy * _T;

		_odo_pose_enc.heading += omega * _T;
	
	}
	
	
	

	memcpy(odo, &_odo_pose_enc, sizeof(pose_t));

    	
    	if (VERBOSE_ODO_ENC)
          	printf("ODO with wheel encoders : %g %g %g\n", odo->x , odo->y , RAD2DEG(odo->heading) );
    	
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

	_T = time_step / 1000.0;
}



