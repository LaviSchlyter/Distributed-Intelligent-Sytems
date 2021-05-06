/*****************************************************************************/
/* File:         project_supervisor.c                                    */
/* Version:      1.0                                                         */
/* Date:         21-Apr-21                                                   */
/* Description: This supervisor writes down the true position of the robots in a 
/* log file.
/*                                                                           */
/* Author: 	 Clément Cosson			     */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

#define FLOCK_SIZE	1 		// Number of robots in flock
#define TIME_STEP	16		// [ms] Length of time step

static WbNodeRef robs[FLOCK_SIZE];              // Robots nodes
static WbFieldRef robs_translation[FLOCK_SIZE]; // Robots translation fields
static WbFieldRef robs_rotation[FLOCK_SIZE];    // Robots rotation fields

//define variables
float t;
float loc[FLOCK_SIZE][3];	// True location of everybody in the flock
float apr_loc[FLOCK_SIZE][3]; //Approximate locations
static FILE *fp = NULL;
// + migration goal x and z

/*
 * Initialize flock position and devices. Use only once at the beginning
 *  of main function.
 */
void reset_robots(void) {
	wb_robot_init();
	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",i);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_translation[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");

	}

	return; 

}

void supervisor_init_log(const char* filename)
{
  //printf("%s", filename);
  fp = fopen(filename,"w");
  
  //Check that the file was opened correctly
  if (fp == NULL){
  printf("Error opening file.\n");
  }
  else {
  printf("Opening file was a success.\n");
  }
  
  //test: write a value
  printf("End of CSV opening.\n");
 
}

void supervisor_print_log(double t)
{
	//Write down true robot positions in a log file
	//Each line will contain the true position of each robot and time
  if( fp != NULL)
  {	
            //write time
            fprintf(fp,"%g,", t);
	  //For each robot, write true x, y and heading
	  for (int i=0;i<FLOCK_SIZE;i++) {
	      fprintf(fp,"%g, %g, %g", loc[i][0], loc[i][1], loc[i][2]);
	      if (i != FLOCK_SIZE){
			  //for all robots except the last, add ','
			  fprintf(fp, ",");
		  }
	  }

	  //Start new line
	  fprintf(fp, "\n");
  }

}

/*
 * Compute performance metric. called at each timestep.
 */
void compute_fitness(float* fit_loc) {
	*fit_loc = 0;
	
	// Compute performance indices for all robots at once.
	int i; 
	for (i=0;i<FLOCK_SIZE;i++) {
		//sqrt((x-xapr)^2 + (z-zapr)^2)
		//x has idx 0, ??y has idx 1??
		*fit_loc = sqrt(pow(loc[i][0]-apr_loc[i][0],2)+pow(loc[i][1]-apr_loc[i][1],2));
	}
}

int main(int argc, char *args[]) {
	
           //Initialize robot position and captors
   	reset_robots();
	
	//Initialize log file where true positions will be written
	char *filename = "Robots_true_position.csv";
	printf("%s\n",filename);
	
           supervisor_init_log(filename);
		
	for(;;) {	//enter infinite loop
		wb_robot_step(TIME_STEP);
		for (int i=0;i<FLOCK_SIZE;i++) {
			// Get true position data for each robot
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0]; // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2]; // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
			//Write down true positions in a log file 
			supervisor_print_log(t);
		}
  		t += TIME_STEP; //update time step
	}
  fflush(fp);
  fclose(fp);
}

/*In main:
define filename = "Apr_loc_rob_x"
supervisor_init_log(filename);
*/


