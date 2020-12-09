""" 
Predicts the subject path from current environment information. 
Subject path consists of trajectory of states
state [6x1]: [x, y, xdot, ydot, xddot, yddot] 
"""

from time import time
import numpy as np
import carla
import copy

class PathPredictor:
    def __init__(self, world, agent, dt):
        self.subject_path = None
        self.world = world
        self.map = self.world.get_map()
        self.vehicle = agent.vehicle
        self.dt = dt
        self.state = np.zeros((6,1))
        self.update_state()

    def update_state(self):
        """
        Update the current state for the vehicle from Carla ground-truth
        """
        loc = self.vehicle.get_location()
        vel = self.vehicle.get_velocity()
        acc = self.vehicle.get_acceleration()
        self.t = self.world.get_snapshot().timestamp.elapsed_seconds

        #state [6x1]: [x, y, xdot, ydot, xddot, yddot] 
        self.state = np.array([[loc.x, loc.y, vel.x, vel.y, acc.x, acc.y]])
        return

    def get_predicted_path(self, time_steps):
        """
        Predict location of the subject for n time_steps
        Args:
            time_steps ([int]): [Number of steps to predict location for]
        """                
        self.update_state()

        dt  = self.dt
        pred_matrix = np.array([[1, 0, dt, 0, 0, 0],
                                [0, 1, 0, dt, 0, 0],
                                [0, 0, 1, 0, dt, 0],
                                [0, 0, 0, 1, 0, dt],
                                [0, 0, 0, 0, 0, 1],
                                [0, 0, 0, 0, 0, 1]])
        state = self.state.T.copy()
        prediction = []

        for i in range(time_steps):
            
            #1. predict next(i*dt) state using point mass kinematic equations (simple extrapolation)
            next_state = pred_matrix@state

            #2. compute distance b/w curr pose and next predicted pose
            pos = np.array((state[0][0], state[1][0]))
            new_pos = np.array((next_state[0][0], next_state[1][0]))
            dist =  np.linalg.norm(pos - new_pos)

            #3. find wp on a curr carla lane some distance away from curr pose
            # will take care of curved roads
            loc = carla.Location(x=pos[0], y=pos[1], z=0)

            if dist > 2:
                # no need to query the map if dist b/e subsquent poses less than half the length of car
                curr_wp = self.map.get_waypoint(loc)
                next_wp = curr_wp.next(dist)
                if(len(next_wp)):
                    new_loc = next_wp[0].transform.location
                else:
                    print("Couldn't find point on the lane, hence extrapolating")
                    new_loc = carla.Location(x=new_pos[0], y=new_pos[1], z=0)                
            else:
                new_loc = carla.Location(x=new_pos[0], y=new_pos[1], z=0)                

            new_t = self.t + (i+1)*dt
            # prediction -> [x, y, x_dot, y_dot, t]
            prediction.append([new_loc.x, new_loc.y, state[2][0], state[3][0], new_t])
            
            state=np.array([new_loc.x, new_loc.y, state[2][0], state[3][0], state[4][0], state[5][0]]).reshape(-1,1)

        return prediction