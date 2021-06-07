/*****************************************************************************************************************************/
/* File:         pso_simplified_sup_flock.c                                                */
/* Version:      1.0                                                            */
/* Date:         01-Mai-21                                                      */
/* Description:  PSO for the avoidance with a public individual heterogeneous   */
/*               configuration. Robot controller based on robot_flocking.c      */
/*                                                                              */
/* Author:       Paco Mermoud                                                   */
/*****************************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <webots/emitter.h>
#include <webots/receiver.h>
#include <webots/supervisor.h>
#include <webots/robot.h>

/* Tunable parameters: ------------------------------------------------------
 NOISY : activate the noise resistance PSO with reevaluation of the best performance
 DOMAIN_WEIGHT : Limit the parameters of the PSO in the domain
 RND_POS : Spawn all the robot in a fixed configuration at different location on the z-axis
 PRIOR_KNOWLEDGE : Initialize the first particles weight to the hand-tune value found empiricaly */

#define NOISY 1
#define DOMAIN_WEIGHT 0
#define RND_POS 1
#define PRIOR_KNOWLEDGE 0

#if NOISY == 1
#define ITS_COEFF 1.0     // Multiplier for number of iterations
#else
#define ITS_COEFF 2.0     // Multiplier for number of iterations
#endif

// Limit Domain
#define MIN_BRAITEN -200.0              // Lower bound on initialization value for braiten
#define MAX_BRAITEN 200.0               // Upper bound on initialization value for braiten

// OTHERS
#define FONT "Arial"
#define FLOCK_SIZE 5                    // Number of robots
#define ROBOTS 1                        // Number of robot with different particles
#define ROB_RAD 0.035                   // Radius of the robots
#define WHEEL_RADIUS 0.0205	            // Wheel radius (meters)
#define MAX_SPEED  6.28                 // Maximum speed of the robots
#define NB_SENSORS 8                    // Number of distance sensors
#define TIME_STEP	64	                // [ms] Length of time step

/* PSO definitions */
#define NB_PARTICLE 9                   // Number of particles in swarm
#define MIN_WEIGHT_BRAITEN -200         // Minimum of a particles weight for braiten
#define MAX_WEIGHT_BRAITEN 200          // Maximum of a particles weight for braiten
#define DATASIZE NB_SENSORS             // Number of elements in particle

// Tune PSO
#define NB_NEIGHBOURS 2                 // Number of neighbors on each side
#define LWEIGHT 2.0                     // Weight of attraction to personal best
#define NBWEIGHT 4.0                    // Weight of attraction to neighborhood best
#define DAMPING 0.6                     // damping of the particle velocity
#define VMAX 30.0                       // Maximum velocity of particles by default
#define ITS_PSO 20                      // Number of iterations to run
#define FINALRUNS 1
#define N_RUNS 1

/* Types of fitness evaluations */
#define EVOLVE 0          // Find new fitness
#define EVOLVE_AVG 1      // Average new fitness into total
#define SELECT 2          // Find more accurate fitness for best selection

#define PI 3.1415926535897932384626433832795 // Number Pi


static WbNodeRef epucks[FLOCK_SIZE];
WbDeviceTag emitter[FLOCK_SIZE];
WbDeviceTag rec[FLOCK_SIZE];
const double *loc[FLOCK_SIZE];
const double *rot[FLOCK_SIZE];
double new_loc[FLOCK_SIZE][3];
double new_rot[FLOCK_SIZE][4];

// Print in webots FONT
char label[50];
char label2[20];

// initial set of weight for pso
double prior_knowledge[DATASIZE] = {17,29,34,10,8,-60,-64,-84 // Braitenberg right
                        //-80,-66,-62,8,10,36,28,18,
                        };

// Relative position of each robots
const double rel_init_pos_robot[FLOCK_SIZE][2]={{-1,0},
                                             {-1,0.1},
                                             {-1,-0.1},
                                             {-1,0.2},
                                             {-1,-0.2},
                                             };


/* RESET - Get device handles and starting locations */
void reset(void) {
  wb_robot_init();
  // Device variables
  char epuck[] = "epuck0";
  char em[] = "emitter0_pso";
  char receive[] = "receiver0_pso";

  int i; //counter
  for (i=0;i<FLOCK_SIZE;i++) {
    epucks[i] = wb_supervisor_node_get_from_def(epuck);
    loc[i] = wb_supervisor_field_get_sf_vec3f(wb_supervisor_node_get_field(epucks[i],"translation"));
    new_loc[i][0] = loc[i][0]; new_loc[i][1] = loc[i][1]; new_loc[i][2] = loc[i][2];
    rot[i] = wb_supervisor_field_get_sf_rotation(wb_supervisor_node_get_field(epucks[i],"rotation"));
    new_rot[i][0] = rot[i][0]; new_rot[i][1] = rot[i][1]; new_rot[i][2] = rot[i][2]; new_rot[i][3] = rot[i][3];
    emitter[i] = wb_robot_get_device(em);
    if (emitter[i]==0) printf("missing emitter %d\n",i);
    rec[i] = wb_robot_get_device(receive);
    wb_receiver_enable(rec[i],TIME_STEP/2);
    epuck[5]++;
    em[7]++;
    receive[8]++;
  }
  wb_robot_step(TIME_STEP*4);
}


// Copy one particle to another
void copyParticle(double particle1[DATASIZE], double particle2[DATASIZE]) {
  int i;                   // FOR-loop counters

  // Copy one bit at a time
  for (i = 0; i < DATASIZE; i++)
    particle1[i] = particle2[i];
}


// Update the best performance of a single particle
void updateLocalPerf(double particles[NB_PARTICLE][DATASIZE],
double perf[NB_PARTICLE], double lbest[NB_PARTICLE][DATASIZE],
double lbestperf[NB_PARTICLE], double lbestage[NB_PARTICLE]) {

  int i;                   // FOR-loop counters

  // If current performance of particle better than previous best, update previous best
  for (i = 0; i < NB_PARTICLE; i++) {
    if (perf[i] > lbestperf[i]) {
      copyParticle(lbest[i],particles[i]);
      lbestperf[i] = perf[i];
      lbestage[i] = 1.0;
    }
  }
}


// Find the best result found, set best to the particle, and return the performance
double bestResult(double lbest[NB_PARTICLE][DATASIZE], double lbestperf[NB_PARTICLE], double best[DATASIZE]) {
  double perf;         // Current best performance
  int i;               // FOR-loop counters

  // Start with the first particle as best
  copyParticle(best,lbest[0]);
  perf = lbestperf[0];


  // Iterate through the rest of the particles looking for better results
  for (i = 1; i < NB_PARTICLE; i++) {
    // If current performance of particle better than previous best, update previous best
    if (lbestperf[i] > perf) {
      copyParticle(best,lbest[i]);
      perf = lbestperf[i];
    }
  }


  return perf;
}


// Update the best performance of a particle neighborhood
void updateNBPerf(double lbest[NB_PARTICLE][DATASIZE], double lbestperf[NB_PARTICLE],
		      double nbbest[NB_PARTICLE][DATASIZE], double nbbestperf[NB_PARTICLE],
		      int neighbors[NB_PARTICLE][NB_PARTICLE]) {
  int i,j;                   // FOR-loop counters

  // For each particle, check the best performances of its neighborhood (-NB to NB, with wraparound from swarmsize-1 to 0)
  for (i = 0; i < NB_PARTICLE; i++) {

    nbbestperf[i] = lbestperf[i];

    for (j = 0; j < NB_PARTICLE; j++) {

      // Make sure it's a valid particle
      if (!neighbors[i][j]) continue;

      // If current performance of particle better than previous best, update previous best
      if (lbestperf[j] > nbbestperf[i]) {
      	copyParticle(nbbest[i],lbest[j]);
      	nbbestperf[i] = lbestperf[j];
      }
    }
  }
}


// Find the modulus of an integer
int mod(int num, int base) {
  while (num >= base)
    num -= base;
  while (num < 0)      // Check for if number is negative to
    num += base;
  return num;
}


// Generate random number in [0,1]
double rnd(void) {
  return ((double)rand())/((double)RAND_MAX);
}


// Randomly position specified robot
double init_pos(int rob_id) {
  static double rnd_posz=0;  // Z-position of the first robot (origin of the group)
  double posz_rob_pso=0;     // Robot's position in the z-axis
  if(rob_id==0){
    rnd_posz = (1.8-ROB_RAD)*rnd() - (1.8-ROB_RAD)/2.0;
  }
  if(!RND_POS){  // Fixed position
    //printf("Setting random position for %d\n",rob_id);
    new_rot[rob_id][0] = 0.0;
    new_rot[rob_id][1] = -1.0;
    new_rot[rob_id][2] = 0.0;
    new_rot[rob_id][3] = 1.5708;
    new_loc[rob_id][1] = 0;
    switch(rob_id){
      case 0:
        new_loc[rob_id][0] = -2.9;
        new_loc[rob_id][2] = 0;
        posz_rob_pso=new_loc[rob_id][2];
        break;
      case 1:
        new_loc[rob_id][0] = -2.9;
        new_loc[rob_id][2] = 0.1;
        posz_rob_pso=new_loc[rob_id][2];
        break;
      case 2:
        new_loc[rob_id][0] = -2.9;
        new_loc[rob_id][2] = -0.1;
        posz_rob_pso=new_loc[rob_id][2];
        break;
      case 3:
        new_loc[rob_id][0] = -2.9;
        new_loc[rob_id][2] = 0.2;
        posz_rob_pso=new_loc[rob_id][2];
        break;
      case 4:
        new_loc[rob_id][0] = -2.9;
        new_loc[rob_id][2] = -0.2;
        posz_rob_pso=new_loc[rob_id][2];
        break;
      default:
        printf("Error in ini_pos in pos_obs_sup, rob_id%d not in range", rob_id);
        break;
     }
   }
   else{  // Rnd position
     new_rot[rob_id][0] = 0.0;
     new_rot[rob_id][1] = -1.0;
     new_rot[rob_id][2] = 0.0;
     new_rot[rob_id][3] = 1.5708;
     new_loc[rob_id][1] = 0;
     new_loc[rob_id][0] = rel_init_pos_robot[rob_id][0];
     new_loc[rob_id][1] = 0.001;
     new_loc[rob_id][2] = rnd_posz + rel_init_pos_robot[rob_id][1];
     posz_rob_pso = rnd_posz + rel_init_pos_robot[rob_id][1];
   }

  // Teleport the robot
  wb_supervisor_field_set_sf_vec3f(wb_supervisor_node_get_field(epucks[rob_id],"translation"), new_loc[rob_id]);
  wb_supervisor_field_set_sf_rotation(wb_supervisor_node_get_field(epucks[rob_id],"rotation"), new_rot[rob_id]);
  return posz_rob_pso;
}


// Send the weights to the robots, continuously compute the metric of the simulation and return the associated fitness
void fitness(double weights[ROBOTS][DATASIZE], double fit[ROBOTS]) {
  double buffer[255];
  double *rbuffer;
  double posz_rob_pso;  // save robot position in the z-axis to send it to the robot controller (for GPS)
  int i,j;            // iterator for-loop
  /* Send data to robots */
  for (i=0;i<FLOCK_SIZE;i++) {
    posz_rob_pso=init_pos(i);
    for (j=0;j<DATASIZE;j++) {
      buffer[j] = weights[i][j];
    }
    buffer[DATASIZE]=posz_rob_pso;
    wb_emitter_send(emitter[i],(void *)buffer,(DATASIZE+1)*sizeof(double));
  }
  wb_supervisor_simulation_reset_physics();

  /* Wait for response */
  printf("Superviser begins Simulation\n");
  while (wb_receiver_get_queue_length(rec[0]) == 0){
    wb_robot_step(TIME_STEP);
  }

   /* Get fitness values from robots */
   for (i=0;i<FLOCK_SIZE;i++) {
      rbuffer = (double *)wb_receiver_get_data(rec[i]);
      fit[i] = rbuffer[0];
      wb_receiver_next_packet(rec[i]);
      printf("\nfit robot%d (%0.2lf) \n", i, fit[i]);
  }
  printf("\n -------------------------------------------------------------------\n");
}


// Find the current performance of the swarm.
// Higher performance is better
void findPerformance(double particles[NB_PARTICLE][DATASIZE], double perf[NB_PARTICLE],
		     double age[NB_PARTICLE], char type) {
  double particles_sim[ROBOTS][DATASIZE]; // Assign 1 particle for each robot of the flock in simulation
  double fit[ROBOTS];
  int i,j,k;                   // FOR-loop counters
  for (i = 0; i < NB_PARTICLE; i+=ROBOTS) {
    for (j=0;j<ROBOTS && i+j<NB_PARTICLE;j++) {
      sprintf(label2,"Particle: %d\n", i+j);
      wb_supervisor_set_label(1,label2,0.01,0.05,0.05,0xffffff,0,FONT);
      for (k=0;k<DATASIZE;k++){
          particles_sim[j][k] = particles[i+j][k];
      }
    }
    // FITNESS FUNCTION
    if (type == EVOLVE_AVG) {
      fitness(particles_sim,fit);
      for (j=0;j<ROBOTS && i+j<NB_PARTICLE;j++) {
      	perf[i+j] = ((age[i+j]-1.0)*perf[i+j] + fit[j])/age[i+j];
      	age[i+j]++;
      }
    }
    else if (type == EVOLVE) {
      fitness(particles_sim,fit);
      for (j=0;j<ROBOTS && i+j<NB_PARTICLE;j++)
	       perf[i+j] = fit[j];
    }
    else if (type == SELECT) {
      for (j=0;j<ROBOTS && i+j<NB_PARTICLE;j++)
          perf[i+j] = 0.0;
      for (k=0;k<5;k++) {
	  fitness(particles_sim,fit);
    	for (j=0;j<ROBOTS && i+j<NB_PARTICLE;j++)
    	  perf[i+j] += fit[j];
      }
      for (j=0;j<ROBOTS && i+j<NB_PARTICLE;j++) {
	       perf[i+j] /= 5.0;
      }
    }
  }
}


/* Particle swarm optimization function */
void limit_weight(double min, double max, double particles[NB_PARTICLE][DATASIZE], int i, int j){
  if (particles[i][j] > max){
    particles[i][j] = max;
  }
  if (particles[i][j] < min){
     particles[i][j] = min;
  }
}


/* Particle swarm optimization function */
void pso(double best_weight[DATASIZE]){
  int i,j,k;                                // FOR-loop counters
  double min, max;                         // Bound for random init of weights
  double particles[NB_PARTICLE][DATASIZE];// Swarm of particles
  double perf[NB_PARTICLE];                 // Current local performance of swarm
  double lbest[NB_PARTICLE][DATASIZE];    // Current best local swarm
  double lbestperf[NB_PARTICLE];            // Current best local performance
  double lbestage[NB_PARTICLE];             // Life length of best local swarm
  double nbbest[NB_PARTICLE][DATASIZE];   // Current best neighborhood
  double nbbestperf[NB_PARTICLE];           // Current best neighborhood performance
  double v[NB_PARTICLE][DATASIZE];        // Velocity of particles
  int neighbors[NB_PARTICLE][NB_PARTICLE];  // Neighbor matrix
  double bestperf;                          // Performance of evolved solution

  // Seed the random generator
  srand(time(NULL));

  // Setup neighborhood (Topological)
  printf("Init neighborhood\n");
  for (i = 0; i < NB_PARTICLE; i++) {
    for (j = 0; j < NB_PARTICLE; j++) {
      if (mod(i-j+NB_NEIGHBOURS, NB_PARTICLE) <= 2*NB_NEIGHBOURS){
        neighbors[i][j] = 1;
      }
      else{
        neighbors[i][j] = 0;
      }
    }
  }

  // Display iteration number
  sprintf(label, "Iteration: 0");
  wb_supervisor_set_label(0,label,0.01,0.01,0.1,0xffffff,0,FONT);

  // Initialize the swarm
  for (i = 0; i < NB_PARTICLE; i++) {
    for (j = 0; j < DATASIZE; j++) {

      // Assign initial value with handtuned weights
      if(PRIOR_KNOWLEDGE){
          // Assign first weight with prior knowledge
          particles[i][j]=prior_knowledge[j];
          lbest[i][j] = particles[i][j];           // Best configurations are initially current configurations
          nbbest[i][j] = particles[i][j];
          v[i][j] = 2.0*VMAX*rnd()-VMAX;         // Random initial velocity
      }

      // Randomly assign initial value in [min,max]
      else{
          min=MIN_BRAITEN;
          max=MAX_BRAITEN;
          particles[i][j] = (max-min)*rnd()+min;
          lbest[i][j] = particles[i][j];           // Best configurations are initially current configurations
          nbbest[i][j] = particles[i][j];
          v[i][j] = 2.0*VMAX*rnd()-VMAX;         // Random initial velocity
      }
    }
  }
  // Best performances are initially current performances
  findPerformance(particles, perf, NULL, EVOLVE);
  for (i = 0; i < NB_PARTICLE; i++) {
    lbestperf[i] = perf[i];
    lbestage[i] = 1.0;  // One performance so far
    nbbestperf[i] = perf[i];
  }

  // Find best neighborhood performances
  updateNBPerf(lbest,lbestperf,nbbest,nbbestperf,neighbors);

  printf("****** Swarm initialized\n");

  // Run optimization
  for (k = 0; k < ITS_COEFF*ITS_PSO; k++) {
    sprintf(label, "Iteration: %d",k+1);
    wb_supervisor_set_label(0,label,0.01,0.01,0.1,0xffffff,0,FONT);
    // Update preferences and generate new particles
    for (i = 0; i < NB_PARTICLE; i++) {
      for (j = 0; j < DATASIZE; j++) {
          v[i][j] *= (double) DAMPING;
          v[i][j] += (double) (LWEIGHT*rnd()*(lbest[i][j] - particles[i][j]) + NBWEIGHT*rnd()*(nbbest[i][j] - particles[i][j]));
          particles[i][j] += v[i][j]; // Move particles


          // Limit of the domain
          if(DOMAIN_WEIGHT){
              limit_weight(MIN_WEIGHT_BRAITEN, MAX_WEIGHT_BRAITEN, particles, i, j);
          }
      }
    }



     // RE-EVALUATE PERFORMANCES OF PREVIOUS BESTS
#if NOISY == 1
    findPerformance(lbest,lbestperf,lbestage,EVOLVE_AVG);
#endif


    // Find new performance
    findPerformance(particles,perf,NULL,EVOLVE);

    // Update best local performance
    updateLocalPerf(particles,perf,lbest,lbestperf,lbestage);

    // Update best neighborhood performance
    updateNBPerf(lbest,lbestperf,nbbest,nbbestperf,neighbors);

    // Find and print the best result of the last PSO_iteration
    double temp[DATASIZE];
    bestperf = bestResult(lbest,lbestperf,temp);
    printf("\n...................................................................................................................\nBest performance of the iteration : %lf",bestperf);
    printf("\nBest Weights of the iteration:");
    for (i=0; i<DATASIZE;i++){
      if(!(i%8)){
         printf("\n%.2lf\t", temp[i]);
      }
      else{
         printf("%.2lf\t", temp[i]);
      }
    }
   printf("\n ...................................................................................................................\n");
   }

   // Find and print the best result of the PSO
   findPerformance(lbest,lbestperf,NULL,SELECT);
   bestperf = bestResult(lbest,lbestperf,best_weight);
   printf("_____Best performance found\n");
   printf("Performance over %d iterations: %lf\n",ITS_PSO,bestperf);

    // Display end of the PSO
   sprintf(label, "Optimization process over.");
   wb_supervisor_set_label(0,label,0.01,0.01,0.1,0xffffff,0,FONT);
}

/*
 * Main function : Run the pso and display the best average performance
*/
int main() {
  reset();
  double best_weight[DATASIZE]; // best solution of pso
  double fit, w[ROBOTS][DATASIZE], f[ROBOTS];
  int i,j,k;  // Counter variables

  // Get result of optimization
  // Do N_RUNS runs and send the best controller found to the robot
  for (j=0;j<N_RUNS;j++) {
    pso(best_weight);

    // Set robot weights to optimization results
    fit = 0.0;
    for (i=0;i<ROBOTS;i++) {
        for (k=0;k<DATASIZE;k++)
            w[i][k] = best_weight[k];
        }

    // Run FINALRUN tests and calculate average
    printf("Running final runs\n");
    for (i=0;i<FINALRUNS;i+=ROBOTS) {
        fitness(w,f);
        for (k=0;k<ROBOTS && i+k<FINALRUNS;k++) {
            //fitvals[i+k] = f[k];
            fit += f[k];
        }
    }
    fit /= FINALRUNS;  // average over the 10 runs

    printf("Average Performance: %.3f\n",fit);
  }

  /* Wait forever */
  while (1){
    fitness(w,f);
  }

  return 0;
}


