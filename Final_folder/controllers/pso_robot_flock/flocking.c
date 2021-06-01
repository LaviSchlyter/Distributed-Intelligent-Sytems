#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/differential_wheels.h>
#include <webots/distance_sensor.h>
#include <webots/emitter.h>
#include <webots/receiver.h>

#define VERBOSE           0
#define NB_SENSORS	     8	  // Number of distance sensors
#define MIN_SENS          350     // Minimum sensibility value
#define MAX_SENS          4096    // Maximum sensibility value
#define MAX_SPEED         800    //800 or 1000 (TODO braitenberg)     // Maximum speed
#define MAX_SPEED_WEB      6.28    // Maximum speed webots
#define FLOCK_SIZE	       5	  // Size of flock
#define TIME_STEP	       64	  // [ms] Length of time step

#define AXLE_LENGTH 		0.052	// Distance between wheels of robot (meters)
#define SPEED_UNIT_RADS	0.00628	// Conversion factor from speed unit to radian per second
#define WHEEL_RADIUS		0.0205	// Wheel radius (meters)
#define DELTA_T		0.064	// Timestep (seconds)

// Reynolds
#define RULE1_THRESHOLD     0.2   // Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        (0.6/10)*1	   // Weight of aggregation rule. default 0.6/10

#define RULE2_THRESHOLD     0.15   // Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        (0.02/10)*1	   // Weight of dispersion rule. default 0.02/10

#define RULE3_WEIGHT        (1.0/10)*0   // Weight of consistency rule. default 1.0/10

#define MIGRATION_WEIGHT    (0.01/10)   // Wheight of attraction towards the common goal. default 0.01/10

#define MIGRATORY_URGE 1 // Tells the robots if they should just go forward or move towards a specific migratory direction


WbDeviceTag left_motor; //handler for left wheel of the robot
WbDeviceTag right_motor; //handler for the right wheel of the robot
WbDeviceTag ds[NB_SENSORS]; // ps[NB_SENSORS];(TODO braitenberg)	// Handle for the infrared distance sensors
WbDeviceTag receiver;		// Handle for the receiver node
WbDeviceTag emitter;		// Handle for the emitter node

int e_puck_matrix[16] = {17,29,34,10,8,-60,-64,-84,
                        -80,-66,-62,8,10,36,28,18}; // for obstacle avoidance

int robot_id_u, robot_id;	// Unique and normalized (between 0 and FLOCK_SIZE-1) robot ID
int robot_verbose;

float relative_pos[FLOCK_SIZE][3];	// relative X, Z, Theta of all robots "par rapport à moi"
float prev_relative_pos[FLOCK_SIZE][3];	// Previous relative  X, Z, Theta values
float my_position[3];     		// X, Z, Theta of the current robot
float prev_my_position[3];  		// X, Z, Theta of the current robot in the previous time step
float speed[FLOCK_SIZE][2];		// Speeds calculated with Reynold's rules
float relative_speed[FLOCK_SIZE][2];	// Speeds calculated with Reynold's rules
int initialized[FLOCK_SIZE];		// != 0 if initial positions have been received
float migr[2] = {25,-25};	        // Migration vector
char* robot_name;
int avoidance=0; // boolean to avoid obstacle
int side=0; // side of avoidance (left=0, right=1)
float theta_robots[FLOCK_SIZE];

/*
 * Reset the robot's devices and get its ID
 */
static void reset() {
	wb_robot_init();
	receiver = wb_robot_get_device("receiver");
	emitter = wb_robot_get_device("emitter");

	//get motors
    left_motor = wb_robot_get_device("left wheel motor");
    right_motor = wb_robot_get_device("right wheel motor");
    wb_motor_set_position(left_motor, INFINITY);
    wb_motor_set_position(right_motor, INFINITY);


	int i;
	char s[4]="ps0";
	for(i=0; i<NB_SENSORS;i++) {
		ds[i]=wb_robot_get_device(s);	// the device name is specified in the world file
		s[2]++;				// increases the device number
	}
	robot_name=(char*) wb_robot_get_name();


	for(i=0;i<NB_SENSORS;i++)
		wb_distance_sensor_enable(ds[i],TIME_STEP);

	wb_receiver_enable(receiver,TIME_STEP/2);

	//Reading the robot's name. Pay attention to name specification when adding robots to the simulation!
	sscanf(robot_name,"epuck%d",&robot_id_u); // read robot id from the robot's name
	robot_id = robot_id_u%FLOCK_SIZE;	  // normalize between 0 and FLOCK_SIZE-1
	
	if(robot_id==0){ // print robot data (ENLEVER)
    	  robot_verbose=1;
	}

	for(i=0; i<FLOCK_SIZE; i++) {
		initialized[i] = 0;		  // Set initialization to 0 (= not yet initialized)
	}

        printf("Reset: robot %d\n",robot_id_u);

        migr[0] = 0;  
        migr[1] = -25;

}


/*
 * Keep given int number within interval {-limit, limit}
 */
void limit(int *number, int limit) {
	if (*number > limit)
		*number = limit;
	if (*number < -limit)
		*number = -limit;
}

/*
 * Updates robot position with wheel speeds
 */
void update_self_motion(int msl, int msr) {
	float theta = my_position[2];

	// Compute deltas of the robot
	float dr = (float)msr * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T; //metres
	float dl = (float)msl * SPEED_UNIT_RADS * WHEEL_RADIUS * DELTA_T;
	float du = (dr + dl)/2.0;
	float dtheta = (dr - dl)/AXLE_LENGTH;

	// Compute deltas in the environment
	float dx = -du * sinf(theta);
	float dz = -du * cosf(theta);

	// Update position
	my_position[0] += dx;
	my_position[1] += dz;
	my_position[2] += dtheta;

	// Keep orientation within 0, 2pi
	if (my_position[2] > 2*M_PI) my_position[2] -= 2.0*M_PI;
	if (my_position[2] < 0) my_position[2] += 2.0*M_PI;
}

/*
 * Computes wheel speed given a certain X,Z speed
 */
void compute_wheel_speeds(int *msl, int *msr) {
	// Compute wanted position from Reynold's speed and current location
	float x = speed[robot_id][0]*cosf(my_position[2]) + speed[robot_id][1]*sinf(my_position[2]); // x in robot coordinates
	float z = -speed[robot_id][0]*sinf(my_position[2]) + speed[robot_id][1]*cosf(my_position[2]); // z in robot coordinates

	float Ku = 0.2;   // Forward control coefficient
	float Kw = 0.5;  // Rotational control coefficient
	float range = sqrtf(x*x + z*z);	  // Distance to the wanted position
	float bearing = -atan2(x, z);	  // Orientation of the wanted position

	// Compute forward control
	float u = Ku*range*cosf(bearing);
	// Compute rotational control
	float w = Kw*bearing;

	// Convert to wheel speeds!
	*msl = (u - AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	*msr = (u + AXLE_LENGTH*w/2.0) * (1000.0 / WHEEL_RADIUS);
	limit(msl,MAX_SPEED);
	limit(msr,MAX_SPEED);
}


/*
 *  Update speed according to Reynold's rules
 */
void reynolds_rules() {

	int i, j, k;			// Loop counters
	float rel_avg_loc[2] = {0,0};	// Flock average positions
	float rel_avg_speed[2] = {0,0};	// Flock average speeds
	float cohesion[2] = {0,0};
	float dispersion[2] = {0,0};
	float consistency[2] = {0,0};


	/* Compute averages over the whole flock */
	for(i=0; i<FLOCK_SIZE; i++) {
                if (i == robot_id)
                    continue; // don't consider yourself for the average
                for (j=0;j<2;j++) {
                         rel_avg_speed[j] += relative_speed[i][j];
                         rel_avg_loc[j] += relative_pos[i][j];
                 }
            }
           for (j=0;j<2;j++) {
                 rel_avg_speed[j] /= FLOCK_SIZE-1;
                 rel_avg_loc[j] /= FLOCK_SIZE-1;
           }

	/* Rule 1 - Aggregation/Cohesion: move towards the center of mass */

      for (j=0;j<2;j++) {

           if (sqrt(pow(rel_avg_loc[0],2)+pow(rel_avg_loc[1],2)) > RULE1_THRESHOLD) {
                cohesion[j] = rel_avg_loc[j];
	 }

      }

	/* Rule 2 - Dispersion/Separation: keep far enough from flockmates */
	for (k=0;k<FLOCK_SIZE;k++) {
		if (k != robot_id) {        // Loop on flockmates only
			// If neighbor k is too close (Euclidean distance)
			if (sqrt(pow(relative_pos[k][0],2)+pow(relative_pos[k][1],2)) < RULE2_THRESHOLD) {
				for (j=0;j<2;j++) {
					dispersion[j] -= 1/(relative_pos[k][j]);	// Relative distance to k
				}
			}
		}
	}




	/* Rule 3 - Consistency/Alignment: match the speeds of flockmates */
	for (j=0;j<2;j++) {
		consistency[j] = rel_avg_speed[j];
		//consistency[j] = 0;
         }

         //aggregation of all behaviors with relative influence determined by weights
	for (j=0;j<2;j++) {
      	      speed[robot_id][j] = cohesion[j] * RULE1_WEIGHT;
  	      speed[robot_id][j] +=  dispersion[j] * RULE2_WEIGHT;
  	      speed[robot_id][j] +=  consistency[j] * RULE3_WEIGHT; 

	}
	speed[robot_id][1] *= -1; //y axis of webots is inverted

	//move the robot according to some migration rule
	if(MIGRATORY_URGE == 0){
	  speed[robot_id][0] += 0.01*cos(my_position[2] + M_PI/2);
	  speed[robot_id][1] += 0.01*sin(my_position[2] + M_PI/2);
	} 
	else {
  	        	speed[robot_id][0] += (migr[0]-my_position[0]) * MIGRATION_WEIGHT;
		speed[robot_id][1] -= (migr[1]-my_position[1]) * MIGRATION_WEIGHT; //y axis of webots is inverted
  	        
	}
}



/*
 *  each robot sends a ping message, so the other robots can measure relative range and bearing to the sender.
 *  the message contains the robot's name
 *  the range and bearing will be measured directly out of message RSSI and direction
*/

void send_ping(void) {
	char out[10];
	strcpy(out,robot_name);  // in the ping message we send the name of the robot.
	wb_emitter_send(emitter,out,strlen(out)+1);
}

/*
 * processing all the received ping messages, and calculate range and bearing to the other robots
 * the range and bearing are measured directly out of message RSSI and direction
*/
void process_received_ping_messages(void) {
	const double *message_direction;
	double message_rssi; // Received Signal Strength indicator
	double theta;
	double range;
	char *inbuffer;	// Buffer for the receiver node
	int other_robot_id;
	while (wb_receiver_get_queue_length(receiver) > 0) {
		inbuffer = (char*) wb_receiver_get_data(receiver);
		message_direction = wb_receiver_get_emitter_direction(receiver);
		message_rssi = wb_receiver_get_signal_strength(receiver);
		double y = message_direction[2];
		double x = message_direction[0];

		theta =	-atan2(y,x);// absolute theta?;why a minus sign?
		theta = theta + my_position[2]; // find the relative theta;
		range = sqrt((1/message_rssi));


		other_robot_id = (int)(inbuffer[5]-'0');  // since the name of the sender is in the received message. Note: this does not work for robots having id bigger than 9!

		// Get position update
		prev_relative_pos[other_robot_id][0] = relative_pos[other_robot_id][0];
		prev_relative_pos[other_robot_id][1] = relative_pos[other_robot_id][1];

		relative_pos[other_robot_id][0] = range*cos(theta);  // relative x pos
		relative_pos[other_robot_id][1] = -1.0 * range*sin(theta);   // relative y pos

		//printf("Robot %s, from robot %d, x: %g, y: %g, theta %g, my theta %g\n",robot_name,other_robot_id,relative_pos[other_robot_id][0],relative_pos[other_robot_id][1],-atan2(y,x)*180.0/3.141592,my_position[2]*180.0/3.141592);

		relative_speed[other_robot_id][0] = relative_speed[other_robot_id][0]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][0]-prev_relative_pos[other_robot_id][0]);
		relative_speed[other_robot_id][1] = relative_speed[other_robot_id][1]*0.0 + 1.0*(1/DELTA_T)*(relative_pos[other_robot_id][1]-prev_relative_pos[other_robot_id][1]);

		wb_receiver_next_packet(receiver);
	}
}



// the main function
int main(){
        printf("Simulation standard");
        int msl, msr;			// Wheel speeds
        float msl_w, msr_w;
        int bmsl, bmsr, sum_sensors;	// Braitenberg parameters
        int i;				// Loop counter
        int ds_value[NB_SENSORS];	// Array for the distance sensor readings
        int max_sens;			// Store highest sensor value
        float priority; // non linear weight to prioritarize braitenberg
        reset();			// Resetting the robot
        msl = 0; msr = 0;
        max_sens = 0;

        // Forever
        for(;;){

            bmsl = 0; bmsr = 0;
            sum_sensors = 0;
            max_sens = 0;
            if(VERBOSE && robot_verbose){printf("-------------------------\n");}
            /* Braitenberg */
            for(i=0;i<NB_SENSORS;i++) {
                ds_value[i]=wb_distance_sensor_get_value(ds[i]); //Read sensor values
                sum_sensors += ds_value[i]; // Add up sensor values
                max_sens = max_sens>ds_value[i]?max_sens:ds_value[i]; // Check if new highest sensor value
                if(VERBOSE&&1&&robot_verbose){printf("robot%d Sensor%d = %d\n", robot_id_u, i,ds_value[i]);}
                // Weighted sum of distance sensor values for Braitenburg vehicle
                bmsr += e_puck_matrix[i] * ds_value[i];//moi 2
                bmsl += e_puck_matrix[i+NB_SENSORS] * ds_value[i];
            }



            // Adapt Braitenberg values (empirical tests)
            bmsl/=MIN_SENS; bmsr/=MIN_SENS;
            bmsl+=66; bmsr+=72;
            if(VERBOSE && 1 && robot_verbose){printf("Bmsl=%d, Bmsr=%d\n", bmsl, bmsr);}
            /* Send and get information */
            send_ping();  // sending a ping to other robot, so they can measure their distance to this robot

            /// Compute self position
            prev_my_position[0] = my_position[0];
            prev_my_position[1] = my_position[1];
            if(VERBOSE && 1 && robot_verbose){printf("My position: x=%lf, y=%lf\n", my_position[0], my_position[1]);}
            update_self_motion(msl,msr);

            process_received_ping_messages();

            speed[robot_id][0] = (1/DELTA_T)*(my_position[0]-prev_my_position[0]);
            speed[robot_id][1] = (1/DELTA_T)*(my_position[1]-prev_my_position[1]);

            // Reynold's rules with all previous info (updates the speed[][] table)
            reynolds_rules();

            // Compute wheels speed from reynold's speed
            if(VERBOSE && 1 && robot_verbose){printf("Avant Reynodl: \nmsl=%d, msr=%d\n", msl, msr);}
            compute_wheel_speeds(&msl, &msr);
            if(VERBOSE && 1 && robot_verbose){printf("Avec Reynodl: \nmsl=%d, msr=%d\n", msl, msr);}


            priority=2;
            if ((ds_value[0]>2100 || ds_value[7]>2100) && !avoidance){ //Entry1 condition of "avoidance
                 avoidance=1;
                 // Avoid to the left or right ?
                 if(ds_value[7] < ds_value[0]){
                   side=0; // left
                   if(robot_verbose){printf("Robot%d to left\n", robot_id_u);}
                 }
                 else{
                   side=1; // right
                   if(robot_verbose){printf("Robot%d to right\n", robot_id_u);}
                 }
            if(robot_verbose){printf("ENTER AVOIDANCE (front) --------------\n");}
            }
            else if ((ds_value[6]>2100 || ds_value[1]>2100) && !avoidance){ //Entry2 condition of "avoidance
                 avoidance=1;
                 // Avoid to the left or right ?
                 if(ds_value[6] < ds_value[1]){
                   side=0; // left
                   if(robot_verbose){printf("Robot%d to left\n", robot_id_u);}
                 }
                 else{
                   side=1; // right
                   if(robot_verbose){printf("Robot%d to right\n", robot_id_u);}
                 }

                if(robot_verbose){printf("ENTER AVOIDANCE (Side) --------------\n");}
            }
            else if ((ds_value[5]>2100 || ds_value[2]>2100) && !avoidance){ //Entry2 condition of "avoidance
               avoidance=1;
               // Avoid to the left or right ?
               if(ds_value[5] < ds_value[2]){
                 side=0; // left
                 if(VERBOSE && 1 && robot_verbose){printf("Robot%d to left\n", robot_id_u);}
               }
               else{
                 side=1; // right
                 if(VERBOSE && 1 && robot_verbose){printf("Robot%d to right\n", robot_id_u);}
               }

                if(VERBOSE && 1 && robot_verbose){printf("ENTER AVOIDANCE (Side) --------------\n");}
            }


            if (!avoidance){ // normal mode
                msl -= msl*max_sens/(priority*MAX_SENS);
                msr -= msr*max_sens/(priority*MAX_SENS);
            }
            else{ // avoidance mode
                if(side){ // avoid to the right
                    if(ds_value[7]>1300){
                        msl = 140;
                        msr = 150;
                    }
                    else{
                        //bmsl=bmsl/2;
                        //bmsr=bmsr/2;
                        msl = 100;
                        msr = 150;
                        if(robot_verbose){printf("Robot%d search wall (right)\n", robot_id_u);}
                    }

                }
                else{ // avoid to the left
                    if(ds_value[0]>1200){
                        msl = 150;
                        msr = 140;
                    }
                    else{
                        //bmsl=bmsl/2;
                        //bmsr=bmsr/2;
                        msl = 150;
                        msr = 100;
                        if(robot_verbose){printf("Robot%d search wall (left)\n", robot_id_u);}
                    }
                }
            }

            if(avoidance && max_sens<70){ //Exit condition of "avoidance
                avoidance=0;
                if(robot_verbose){printf("LEAVE AVOIDANCE---------------\n");}
            }

            if(VERBOSE && 1 && robot_verbose){printf("Avec Priority: \nmsl=%d, msr=%d\n", msl, msr);}
            // Add Braitenberg
            if(VERBOSE && 1 && robot_verbose){printf("Avant addition:\nBmsl=%d, Bmsr=%d\n", bmsl, bmsr);}
            msl += bmsl;
            msr += bmsr;
            limit(&msl,MAX_SPEED);
	 limit(&msr,MAX_SPEED);
            if(VERBOSE && 1 && robot_verbose){printf("Final: \nmsl=%d, msr=%d\n", msl, msr);}
            // Set speed
            msl_w = msl*MAX_SPEED_WEB/(MAX_SPEED+1);
            msr_w = msr*MAX_SPEED_WEB/(MAX_SPEED+1);
            if(VERBOSE && 1 && robot_verbose){printf("Speed: \nmsl_w=%lf, msr_w=%lf\n", msl_w, msr_w);}
            wb_motor_set_velocity(left_motor, msl_w);
            wb_motor_set_velocity(right_motor, msr_w);

            // Continue one step
            wb_robot_step(TIME_STEP);
            }

}