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
        self.agent = agent
        self.dt = dt
        self.state = np.zeros((6,1))
        self.update_state()

    def update_state(self):
        """
        Update the current state for the vehicle from Carla ground-truth
        """
        loc = self.agent.vehicle.get_location()
        vel = self.agent.vehicle.get_velocity()
        acc = self.agent.vehicle.get_acceleration()

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
            
            state = pred_matrix@state

            prediction.append(carla.Location(x=state[0][0], y=state[1][0], z=0.0))
            
        return prediction
