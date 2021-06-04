

------------------------------- LOCALIZATION CONTROLLERS -----------------------------

------------------------------- FLOCKING CONTROLLER ----------------------------------

------------------------------- FORMATION CONTROLLERS --------------------------------

The leader-follower architecture is implemented to move in formation toward a common goal. In each flock, there is always one leader and the other robots are followers. 
For the leader, one controller has been implemented for each world (crossing_leader, obstacle_leader) 
.For the followers, two controllers have been implemented for each world (crossing_follower_laplacian, crossing_follower_mataric, obstacle_follower_laplacian, obstacle_follower_mataric).
Each one of these controllers has several functionalities, that will be described for each world below: 

-- "Obstacles" and "test_obstacles" world -- mataric controller:
1) Assign the controller "obstacle_leader" to robot ID 0 and set MIGRATION_WEIGHT to 0.007 at line 38. The migration urge can be changed at line 83.
2) Assign the controller "obstacle_follower_mataric" to robot ID 1,2,3 and 4.
3) The flock size can be changed as follows:
    - To have a flock size of 7 : add two robots to the world (copy-paste a robot) and place them somewhere in front of the flock, change their DEF and their name to "epuck5" and "epuck6", and change FLOCK_SIZE to 7 in "obstacle_follower_mataric" at line 30.
    - To have a flock size of 6 : add a robot to the world (copy-paste a robot) and place it somewhere in front of the flock, change its DEF and its name to "epuck5", and change FLOCK_SIZE to 6 in "obstacle_follower_mataric" at line 30.
    - To have a flock size of 5 : don't do anything, that's the default size.
    - To have a flock size of 4 : remove robot ID 4 from the world and change FLOCK_SIZE to 4 in "obstacle_follower_mataric" at line 30.
    - To have a flock size of 3 : remove robot ID 4, 3 from the world and change FLOCK_SIZE to 3 in "obstacle_follower_mataric" at line 30.
    - To have a flock size of 2 : remove robot ID 4, 3, 2 from the world and change FLOCK_SIZE to 2 in "obstacle_follower_mataric" at line 30.
4) The controller type can be changed at line 36 of "obstacle_follower_mataric" by changing CONTROLLER_TYPE to P (proportional), PI (proportional and integral) or NPI (non linear proportional and integral). The default controller is proportional.


-- "Obstacles" and "test_obstacles" world  -- laplacian controller:
1) Assign the controller "obstacle_leader" to robot ID 0 and set MIGRATION_WEIGHT to 0.003 at line 38. The migration urge can be changed at line 83.
2) Assign the controller "obstacle_follower_laplacian" to robot ID 1,2,3 and 4.
3) With this controller you can change the number of edges in the graph by changing NB_EDGES (4,8 or 10) for a fixed FLOCK_SIZE of 5. Here are the possible combinations (change the values accordingly at line 35-36 of "obstacle_follower_laplacian"):
    - (FLOCK_SIZE = 5, NB_EDGES = 4)
    - (FLOCK_SIZE = 5, NB_EDGES = 8)
    - (FLOCK_SIZE = 5, NB_EDGES = 10)
4) With this controller you can change the number of robots in the formation by changing FLOCK_SIZE (3,4 or 5) for a fully connected graph. Here are the possible combinations (change the values accordingly at line 35-36 of "obstacle_follower_laplacian"):
    - (FLOCK_SIZE = 3, NB_EDGES = 3) and remove robot ID 4 and 3 from the world.
    - (FLOCK_SIZE = 4, NB_EDGES = 6) and remove robot ID 4 from the world.
    - (FLOCK_SIZE = 5, NB_EDGES = 10)


-- "crossing" world -- mataric controller:
1) Assign the controller "crossing_leader" to robot ID 0 and 5. Change WORLD to CROSSING at line 38.
2) Assign the controller "crossing_follower_mataric" to robot ID 1,2,3,4,6,7,8 and 9. Change WORLD to CROSSING at line 41.
3) The flock size can be changed as follows:
    - To have a flock size of 5 : don't do anything, that's the default size.
    - To have a flock size of 4 : remove robot ID 3 and 9 from the world and change FLOCK_SIZE to 4 in "crossing_follower_mataric" at line 30 and in "crossing_leader" at line 33.
    - To have a flock size of 3 : remove robot ID 3, 4, 8 and 9 from the world and change FLOCK_SIZE to 3 in "crossing_follower_mataric" at line 30 and in "crossing_leader" at line 33.
    - To have a flock size of 2 : remove robot ID 2, 3, 4, 6, 8 and 9 from the world and change FLOCK_SIZE to 2 in "crossing_follower_mataric" at line 30 and in "crossing_leader" at line 33.
4) The controller type can be changed at line 36 of "crossing_follower_mataric" by changing CONTROLLER_TYPE to P (proportional), PI (proportional and integral) or NPI (non linear proportional and integral). The default controller is proportional.

-- "test_crossing world" -- mataric controller:
1) Assign the controller "crossing_leader" to robot ID 0 and 5. Change WORLD to TEST_CROSSING at line 38.
2) Assign the controller "crossing_follower_mataric" to robot ID 1,2,3,4,6,7,8 and 9. Change WORLD to TEST_CROSSING at line 41.
3) The flock size can be changed as follows:
    - To have a flock size of 5 : don't do anything, that's the default size.
    - To have a flock size of 4 : remove robot ID 3 and 8 from the world and change FLOCK_SIZE to 4 in "crossing_follower_mataric" at line 30 and in "crossing_leader" at line 33.
    - To have a flock size of 3 : remove robot ID 3, 4, 8 and 9 from the world and change FLOCK_SIZE to 3 in "crossing_follower_mataric" at line 30 and in "crossing_leader" at line 33.
    - To have a flock size of 2 : remove robot ID 2, 3, 4, 7, 8 and 9 from the world and change FLOCK_SIZE to 2 in "crossing_follower_mataric" at line 30 and in "crossing_leader" at line 33.
4) The controller type can be changed at line 36 of "crossing_follower_mataric" by changing CONTROLLER_TYPE to P (proportional), PI (proportional and integral) or NPI (non linear proportional and integral). The default controller is proportional.


-- "crossing" world -- laplacian controller:
1) Assign the controller "crossing_leader" to robot ID 0 and 5. Change WORLD to CROSSING at line 38.
2) Assign the controller "crossing_follower_laplacian" to robot ID 1,2,3,4,6,7,8 and 9. Change WORLD to CROSSING at line 41.
3) With this controller you can change the number of edges in the graph by changing NB_EDGES (4,8 or 10) for a fixed FLOCK_SIZE of 5. Here are the possible combinations (change the values accordingly at line 35-36 of "crossing_follower_laplacian"):
    - (FLOCK_SIZE = 5, NB_EDGES = 4)
    - (FLOCK_SIZE = 5, NB_EDGES = 8)
    - (FLOCK_SIZE = 5, NB_EDGES = 10)
4) With this controller you can change the number of robots in the formation by changing FLOCK_SIZE (3,4 or 5) for a fully connected graph. Here are the possible combinations (change the values accordingly at line 35-36 of "crossing_follower_laplacian"):
    - (FLOCK_SIZE = 3, NB_EDGES = 3) and remove robot ID 3, 4, 8 and 9 from the world, and change FLOCK_SIZE to 3 in "crossing_leader" at line 33.
    - (FLOCK_SIZE = 4, NB_EDGES = 6) and remove robot ID 4 and 8 from the world, and change FLOCK_SIZE to 4 in "crossing_leader" at line 33.
    - (FLOCK_SIZE = 5, NB_EDGES = 10) 


-- "test_crossing" world -- laplacian controller:
1) Assign the controller "crossing_leader" to robot ID 0 and 5. Change WORLD to TEST_CROSSING at line 38.
2) Assign the controller "crossing_follower_laplacian" to robot ID 1,2,3,4,6,7,8 and 9. Change WORLD to TEST_CROSSING at line 41.
3) With this controller you can change the number of edges in the graph by changing NB_EDGES (4,8 or 10) for a fixed FLOCK_SIZE of 5. Here are the possible combinations (change the values accordingly at line 35-36 of "crossing_follower_laplacian"):
    - (FLOCK_SIZE = 5, NB_EDGES = 4)
    - (FLOCK_SIZE = 5, NB_EDGES = 8)
    - (FLOCK_SIZE = 5, NB_EDGES = 10)
4) With this controller you can change the number of robots in the formation by changing FLOCK_SIZE (3,4 or 5) for a fully connected graph. Here are the possible combinations (change the values accordingly at line 35-36 of "crossing_follower_laplacian"):
    - (FLOCK_SIZE = 3, NB_EDGES = 3) and remove robot ID 3, 4, 8 and 9 from the world, and change FLOCK_SIZE to 3 in "crossing_leader" at line 33.
    - (FLOCK_SIZE = 4, NB_EDGES = 6) and remove robot ID 4 and 9 from the world, and change FLOCK_SIZE to 4 in "crossing_leader" at line 33.
    - (FLOCK_SIZE = 5, NB_EDGES = 10) 


------------------------------------------ PSO ---------------------------------------