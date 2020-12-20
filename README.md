# Planning_Final_Project
This is the repo for the planning final project on socially compliant autonomous vehicles.

Download CARLA on your local machine first.

Download the AdditionalMaps from here(https://github.com/carla-simulator/carla/releases), since we're using Town06.
Put the zip file in to the "Import" folder, then run "./ImportAssets.sh" to import the maps.

Start the simulator by running /opt/carla-simulator/bin/CarlaUE4.sh

To run the system in the default scenario, do:
1. cd lane_change
2. python play.py

The default scenario is where the subject vehicle has the "Very Aggressive" behaviour.



The files inside the lane_change folder have the following functionality:

1. change_to_Town06.py : Functionlaity to change CARLA world to Town06.
2. collision_checker.py : Tooling to check for collisions during coarse trajectory generation.
3. lattice_generator.py : The search (Dijkstra) and backtracking functionality to generate a coarse trajectory to reach the best goal state.
4. path_predictor.py: Tooling to predict the state of the subject vehicle for a given number of seconds in the future.
5. trajectory_generator.py: Functionality to densify the coarse path plan.
5. PID_controller.py: This code has been taken directly from CARLA's repository. Contains code for both the throttle and steer PID controllers.
6. play.py : Code to pipeline all the components of the system together. 







