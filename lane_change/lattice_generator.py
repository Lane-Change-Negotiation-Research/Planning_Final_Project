
""" Lattice Generator will generate lattice graphs for the collision checker """

import numpy as np
import math
import carla
from matplotlib import pyplot as plt
import random
import copy
from more_itertools import pairwise
from collections import deque


class Road:
    
    def __init__(self, road_dimensions = {"left":3.5, "center":0, "right":-3.5}, road_length = 230):
        
        self.road_dimensions = road_dimensions
        self.road_length = road_length        
    
        
    def plot(self):
        
        x = np.arange(0, self.road_length, 0.01)
        y_left = np.ones(len(x)) * self.road_dimensions["left"]
        y_right = np.ones(len(x)) * self.road_dimensions["right"]
        y_center = np.ones(len(x)) * self.road_dimensions["center"]
        
        plt.plot(x,y_left, 'k')
        plt.plot(x,y_right, 'k')
        plt.plot(x,y_center, 'y')


class Actions:
    
    def __init__(self):
        
        self.acc_params = {"deltaT":1, "deltaV":3, "deltaDDiff":[1.5,0]}
        self.const_params = {"deltaT":1, "deltaV":0, "deltaDDiff":[0,0]}
        self.dec_params = {"deltaT":1, "deltaV":-3, "deltaDDiff":[-1.5,0]}
        self.lane_change_params = {"deltaT":4.5, "deltaV":0, "deltaDDiff":[0,3.5]}
        
        self.action_params_list = [self.acc_params, self.const_params, self.dec_params, self.lane_change_params]
        
    def random_action(self, allow_lane_change = True):
        
        if(allow_lane_change == False):
            next_action_idx = np.random.randint(3)
            return False, self.action_params_list[next_action_idx]
        
        next_action_idx = np.random.randint(4)
        if(next_action_idx == 3):
            return True, self.action_params_list[next_action_idx]
        else:
            return False, self.action_params_list[next_action_idx]


class Constraints:
    
    def __init__(self, max_v=60, min_v=15, max_position_x=10000, max_position_y=3.4, min_position_y=-3.4):
        
        self.max_v = max_v
        self.min_v = min_v
        self.max_position_x = max_position_x
        self.max_position_y = max_position_y
        self.min_position_y = min_position_y


class State:
    
    def __init__(self, position = [0,0], speed = 0, time = 0):
        self.position = position
        self.speed = speed
        self.time = time
        
    def apply_constraints(self, constraints):
        
        self.speed = max(min(self.speed, constraints.max_v), constraints.min_v)
        self.position[0] = min(self.position[0], constraints.max_position_x)
        self.position[1] = max(min(self.position[1], constraints.max_position_y), constraints.min_position_y)
        
    def check_termination(self, termination_conditions):
        
        if(self.position[0] > termination_conditions.max_position_x or self.time > termination_conditions.max_time):
            return 1
        return 0

    def collide(self, subject_path):

        pass


class TerminationConditions:
    
    def __init__(self, max_time=100, max_position_x=200):
        
        self.max_time = max_time
        self.max_position_x = max_position_x


class CostCalculator:

  def __init__(self, weights = [1,1,1,1]):

    self.weights = weights

  def _cost_distance(self, initial_state, next_state):

    return np.sqrt((initial_state.position[0] - next_state.position[0])**2 + (initial_state.position[1] - next_state.position[1])**2)

  def _cost_lateral_offset(self, initial_state, next_state):

    return (next_state.position[1] + initial_state.position[1])/2

  def _cost_longitudianl_velocity(self, initial_state, next_state, v_horizon):

    return v_horizon - (initial_state.speed + next_state.speed)/2

  def _cost_longitudinal_acceleration(self, initial_state, next_state):

    acc = (next_state.speed - initial_state.speed) / 1 # DeltaT is 1. TODO: Remove hardcode

    return acc
  
  def compute_trajectory_cost(self, trajectory):

    cost = 0

    for (initial_state, next_state) in pairwise(trajectory):

      cost += self.weights[0] * self._cost_distance(initial_state, next_state)
      cost += self.weights[1] * self._cost_lateral_offset(initial_state, next_state)
      cost += self.weights[2] * self._cost_longitudianl_velocity(initial_state, next_state, 60)
      cost += self.weights[3] * self._cost_longitudinal_acceleration(initial_state, next_state)

    return cost

  def compute_transition_cost(self, initial_state, next_state):

    cost = 0
    cost += self.weights[0] * self._cost_distance(initial_state, next_state)
    cost += self.weights[1] * self._cost_lateral_offset(initial_state, next_state)
    cost += self.weights[2] * self._cost_longitudianl_velocity(initial_state, next_state, 60)
    cost += self.weights[3] * self._cost_longitudinal_acceleration(initial_state, next_state)

    return cost


class CostCalculatorExtended:

  def __init__(self, weights = [1,1,1,1]):

    self.weights = weights

  def _cost_distance(self, initial_state, next_state):

    return np.sqrt((initial_state.position[0] - next_state.position[0])**2 + (initial_state.position[1] - next_state.position[1])**2)

  def _cost_lateral_offset(self, initial_state, next_state):

    return (next_state.position[1] + initial_state.position[1])/2

  def _cost_longitudianl_velocity(self, initial_state, next_state, v_horizon):

    return v_horizon - (initial_state.speed + next_state.speed)/2

  def _cost_longitudinal_acceleration(self, initial_state, next_state):

    acc = (next_state.speed - initial_state.speed) / 1 # DeltaT is 1. TODO: Remove hardcode

    return acc
  
  def compute_trajectory_cost(self, trajectory):

    cost = 0

    for (initial_state, next_state) in pairwise(trajectory):

      cost += self.weights[0] * self._cost_distance(initial_state, next_state)
      cost += self.weights[1] * self._cost_lateral_offset(initial_state, next_state)
      cost += self.weights[2] * self._cost_longitudianl_velocity(initial_state, next_state, 60)
      cost += self.weights[3] * self._cost_longitudinal_acceleration(initial_state, next_state)

    return cost

  def compute_transition_cost(self, initial_state, next_state):

    cost = 0
    cost += self.weights[0] * self._cost_distance(initial_state, next_state)
    cost += self.weights[1] * self._cost_lateral_offset(initial_state, next_state)
    cost += self.weights[2] * self._cost_longitudianl_velocity(initial_state, next_state, 60)
    cost += self.weights[3] * self._cost_longitudinal_acceleration(initial_state, next_state)

    return cost

  def compute_speed_limit_cost(self,initial_state,next_state,targets_state,targets_next_state):
    
    cost = 0
    abs_diff = abs(targets_next_state.speed - next_state.speed)
    cost_diff = 15 #TODO: Remove hardcode 15 m/s above or below speed of other car is bad
    if(abs_diff > cost_diff): 
      cost += self.weights[2] * abs_diff-cost_diff
    
    return cost

  def compute_obsticle_inflation_cost(self,initial_state,next_state,targets_state,targets_next_state):

    cost = 0
    lateral_offset_diff = abs(targets_next_state.position[1] - next_state.position[1])
    if(lateral_offset_diff < 3): #checks if the target and ego vehicle are in the same lane TODO: remove hardcode
      distance_diff = abs(targets_next_state.position[0] - next_state.position[0])
      if((next_state.speed*3)>(distance_diff)):
        cost = self.weights[0] * (next_state.speed*3)/(distance_diff) #ramp - follows 3 second rule - TODO: needs proper scaling
    
    return cost

  def compute_blindspot_cost(self,initial_state,next_state,targets_state,targets_next_state):

    max_dist = 4
    cost = 0
    lateral_offset_diff = abs(targets_next_state.position[1] - next_state.position[1])
    if(lateral_offset_diff > 3 and lateral_offset_diff < 6): #checks if the target and ego vehicle are in the adjacent lanes TODO: remove hardcode
      distance_diff = abs(targets_next_state.position[0]-2 - next_state.position[0]) #the blindspot resides when the front of the ego vehicle is immediately behind the driver of the target vehicle
      if(distance_diff<max_dist):
        cost = self.weights[0] * (max_dist-distance_diff)/max_dist #linear ramp - average car is 4 meters long, should add cost for the length of the car 
    
    return cost


class LatticeGenerator:
    
    def __init__(self, road: Road, actions: Actions, constraints: Constraints, termination_conditions: TerminationConditions, cost_calculator: CostCalculator, start_state: State, obstacles = [], subject_path = []):
        
        self.road = road
        self.obstacles = []
        self.actions = actions
        self.start_state = start_state
        self.constraints = constraints
        self.termination_conditions = termination_conditions
        self.cost_calculator = cost_calculator
        self.subject_path = subject_path
        
        
    def sample_random_trajectory(self):
        
        tmp_state = copy.deepcopy(self.start_state)
        trajectory = [self.start_state]
        ALLOW_LANE_CHANGE = True
        
        while(True):
                    
            #1. Randomly select an action
            is_next_action_lane_change, next_action_params = self.actions.random_action(ALLOW_LANE_CHANGE)
            if(is_next_action_lane_change):
                ALLOW_LANE_CHANGE=False

            #2. Apply action + constraints
            #2.a T
            tmp_state.time += next_action_params["deltaT"]

            #2.b V
            tmp_state.speed += next_action_params["deltaV"]
            tmp_state.apply_constraints(self.constraints)

            #2.c D
            tmp_state.position[0] += (tmp_state.speed + next_action_params["deltaDDiff"][0])
            tmp_state.position[1] += next_action_params["deltaDDiff"][1]
            tmp_state.apply_constraints(self.constraints)
            
            #3. Append to trajectory
            trajectory.append(copy.deepcopy(tmp_state))

            #4. Check if terminal
            if(tmp_state.check_termination(self.termination_conditions)):
                break
                
        return trajectory
    
    def sample_and_plot_random_trajectory(self):
        
        random_trajectory = self.sample_random_trajectory()
        
        # Plot the road
        self.road.plot()
        
        # Plot the states of the random trajectory
        x_vals = []
        y_vals = []
        for state in random_trajectory:
            plt.plot(state.position[0], state.position[1], 'o')
            x_vals.append(state.position[0])
            y_vals.append(state.position[1])
        
        plt.plot(x_vals, y_vals)

        return random_trajectory


    def generate_full_lattice(self, start_state, actions):

        lattice = {} # (x,y,v,t,lane_change_status) -> {"action" -> next (x,y,v,t,lane_change_status)}
        state_2_cost = {}

        queue = deque()
        queue.append((start_state,0, 0)) # (state, has lane change happened, cost)

        while(len(queue) != 0):

          curr_state, has_lane_change_happend, cost = queue.popleft()
          curr_state_tuple = (curr_state.position[0], curr_state.position[1], curr_state.speed, curr_state.time, has_lane_change_happend)

          if(curr_state_tuple in lattice.keys() and state_2_cost[curr_state_tuple] <= cost):
            continue
          lattice[curr_state_tuple] = {}
          state_2_cost[curr_state_tuple] = cost

          for i,next_action_params in enumerate(actions.action_params_list):

            if(i == 3 and has_lane_change_happend):
              continue

            tmp_state = copy.deepcopy(curr_state)

            #Apply action + constraints
            #a T
            tmp_state.time += next_action_params["deltaT"]

            #b V
            tmp_state.speed += next_action_params["deltaV"]
            tmp_state.apply_constraints(self.constraints)

            #c D
            tmp_state.position[0] += (tmp_state.speed + next_action_params["deltaDDiff"][0])
            tmp_state.position[1] += next_action_params["deltaDDiff"][1]
            tmp_state.apply_constraints(self.constraints)

            #Check collision
            if (tmp_state.collide(self.subject_path)):
                continue

            if (i == 3 or has_lane_change_happend):
              tmp_state_tuple = (tmp_state.position[0], tmp_state.position[1], tmp_state.speed, tmp_state.time, 1)
            else:
              tmp_state_tuple = (tmp_state.position[0], tmp_state.position[1], tmp_state.speed, tmp_state.time, 0)

            lattice[curr_state_tuple][i] = tmp_state_tuple

            cost_of_transition = self.cost_calculator.compute_transition_cost(curr_state, tmp_state)

            #Check if terminal
            if(tmp_state.check_termination(self.termination_conditions)):
                continue

            #Add to queue

            if(i == 3 or has_lane_change_happend):
              queue.append((tmp_state,1,cost+cost_of_transition))
            else:
              queue.append((tmp_state,0,cost+cost_of_transition))



        return lattice, state_2_cost


    def plot_some_trajectories(self, lattice, start_state_tuple, k=10):

        self.road.plot()

        for i in range(k):

          curr_state_tuple = copy.deepcopy(start_state_tuple)
          traj = [curr_state_tuple]
          while(curr_state_tuple in lattice.keys()):

            next_action = random.choice([k for k in range(len(lattice[curr_state_tuple]))])
            next_state_tuple = lattice[curr_state_tuple][next_action]

            traj.append(next_state_tuple)
            curr_state_tuple = copy.deepcopy(next_state_tuple)

          x_vals = [p[0] for p in traj]
          y_vals = [p[1] for p in traj]
          plt.plot(x_vals, y_vals)
          for i in range(len(x_vals)):
            plt.plot(x_vals[i], y_vals[i], 'o')

        return 0
       
if __name__ == "__main__":
	road = Road()
	actions = Actions()
	constraints = Constraints()
	termination_conditions = TerminationConditions()
	start_state = State([0,-1.75], 18, 0)
	cost_calculator = CostCalculator()
	obstacles = []


	latticeGenerator = LatticeGenerator(road = road,\
										actions = actions,\
										constraints= constraints,\
										cost_calculator = cost_calculator,\
										termination_conditions=termination_conditions,\
										start_state=start_state,\
										obstacles=obstacles)


	full_lattice, state_2_cost = latticeGenerator.generate_full_lattice(start_state, actions)

	# Example of full lattice access. 
	print("Start State:" , (start_state.position[0], start_state.position[1], start_state.speed, start_state.time, 0))
	print("Next states (Children)", full_lattice[(start_state.position[0], start_state.position[1], start_state.speed, start_state.time, 0)])

	# Full_Lattice has state tuples as the keys.
	# The value associated with each of these keys is a dict, which stores the next_state that will be achieved when applying action 0, 1, 2, 3 etc. In our example, 0 is acc, 1 is dec, 2 is constant speed, 3 is lane change.

	# state : (x, y, speed, time, has_lane_change_happened)

	# Example of state_2_cost
	state_2_cost[(18, 1.75, 18, 4.5, 1)]

	latticeGenerator.plot_some_trajectories(full_lattice, (start_state.position[0], start_state.position[1], start_state.speed, start_state.time, 0), 5)

	random_trajectory = latticeGenerator.sample_and_plot_random_trajectory()
	cost_of_random_trajectory = cost_calculator.compute_trajectory_cost(random_trajectory)
	print("Cost of this trajectory:", cost_of_random_trajectory)