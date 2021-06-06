/*****************************************************************************/
/* File:         project_supervisor.c                                        */
/* Version:      1.0                                                         */
/* Date:         21-Apr-21                                                   */
/* Description: This supervisor writes down the true position of the robots  */
/* in a log file. This is used later on to compute fitness metrics.                                                                */
/*                                                                           */
/* Author: 	 Clément Cosson			            */
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <webots/robot.h>
#include <webots/emitter.h>
#include <webots/supervisor.h>

//These parameters must be adapted for each flock and world
#define FLOCK_SIZE	5 		// Total number of robots in simulation

// This line must be uncommented for all simulaltions in the obstacle or localization world or for crossing with 2 teams of 5 or more robots each 
static int robot_id[14] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13}; //names of robots: epuck%d
//Else, use:
// - For crossing with 2 robots in each team:
//static int robot_id[14] = {0,1,5,6}; //names of robots: epuck%d
//- For crossing with 3 robots in each team:
//static int robot_id[14] = {0,1,2,5,6,7}; //names of robots: epuck%d
//- For crossing with 4 robots in each team:
//static int robot_id[14] = {0,1,2,4,5,6,7,9}; //names of robots: epuck%d


#define TIME_STEP	16		// [ms] Length of time step

static WbNodeRef robs[FLOCK_SIZE];              // Robots nodes
static WbFieldRef robs_translation[FLOCK_SIZE]; // Robots translation fields
static WbFieldRef robs_rotation[FLOCK_SIZE];    // Robots rotation fields

//define variables
float t = 0;
float loc[FLOCK_SIZE][3];	// True location of each robot in the flock
static FILE *fp = NULL;
static double time_step;

/*
 * Initialize robots devices. 
 */
void reset_robots(void) {
	wb_robot_init();
	time_step = wb_robot_get_basic_time_step()/1000; //convert to second (for use in log file)
	char rob[7] = "epuck0";
	int i;
	for (i=0;i<FLOCK_SIZE;i++) {
		sprintf(rob,"epuck%d",robot_id[i]);
		robs[i] = wb_supervisor_node_get_from_def(rob);
		robs_translation[i] = wb_supervisor_node_get_field(robs[i],"translation");
		robs_rotation[i] = wb_supervisor_node_get_field(robs[i],"rotation");
                      sscanf(rob, "epuck%d", &robot_id[i]);
                      printf("rob number %d initialized in supervisor\n",robot_id[i]);
	}

	return; 

}

// Initialize the log file 
void supervisor_init_log(const char* filename)
{
  fp = fopen(filename,"w");
  
  //Check that the file was opened correctly
  if (fp == NULL){
  printf("Error opening file.\n");
  }
  else {
  printf("Opening file was a success.\n");
  }
 
  
  //print header values
  fprintf(fp, "time;"); 
  for(int i=0;i<FLOCK_SIZE;i++)
  {
    fprintf(fp, "true_x_rob%d; true_y_rob%d; true_heading_rob%d", robot_id[i], robot_id[i], robot_id[i]); 
     if (i != FLOCK_SIZE-1){
	//for all robots except the last, add ';'
	fprintf(fp, ";");
		  }
  }
  fprintf(fp, "\n"); 

 
}

void supervisor_print_log()
{
	//Write down true robot positions in a log file
	//i corresponds to current robot
	//Each line will contain the true position of each robot and time
  if( fp != NULL)
    {	
             //Write time at the beginning of the line
	  fprintf(fp,"%g;",t);
	  //For each robot, write true x, y and heading
	  for (int i=0;i<FLOCK_SIZE;i++) {
	      fprintf(fp,"%g; %g; %g", loc[i][0], loc[i][1], loc[i][2]);
	      //printf(fp,"%g; %g; %g", loc[i][0], loc[i][1], loc[i][2]);
	      if (i != FLOCK_SIZE-1){
			  //for all robots except the last, add ','
			  fprintf(fp, ";");
		  }
                  else{
	  //Start new line if last robot
        	  fprintf(fp, "\n");
	  }
	  }


  }

}

int main(int argc, char *args[]) {
	
           //Initialize robot position and captors
   	reset_robots();
	
	//Initialize log file where true positions will be written
           char filename[64];
           sprintf(filename, "supervisor_log.csv");
	printf("%s\n",filename);
	
           supervisor_init_log(filename);
		
	for(;;) {	//enter infinite loop
		wb_robot_step(TIME_STEP);
		for (int i=0;i<FLOCK_SIZE;i++) {
			// Get true position data for each robot
			loc[i][0] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[0]; // X
			loc[i][1] = wb_supervisor_field_get_sf_vec3f(robs_translation[i])[2]; // Z
			loc[i][2] = wb_supervisor_field_get_sf_rotation(robs_rotation[i])[3]; // THETA
			
		}
		//Write down true positions in a log file 
		supervisor_print_log();
		t += time_step; //update time
	}
  fflush(fp);
  fclose(fp);
}



