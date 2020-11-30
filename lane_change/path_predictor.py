""" 
Predicts the subject path from current environment information. 
Subject path consists of trajectory of states

state [6x1]: [x, y, xdot, ydot, xddot, yddot] 
"""

import numpy as np
import carla

class PathPredictor:
    def __init__(self, world, agent_id, dt):
        self.subject_path = None
        self.world = world
        self.agent = self.world.get_actor(agent_id)
        self.dt = dt
        self.state = np.zeros((6,1))

    def get_gt_path(self):

        # TODO: return the path from ground truth state
        # This is not possible as we can't access path buffer inside traffic manager
        # from client side
        
        return

    def update_state():
        """
        Update the current state for the vehicle from Carla ground-truth
        """
        location = self.agent.get_location()
        velocity = self.agent.get_velocity()
        acceleration =self.agent.get_acceleration()
        # TODO: update the state using above three updates

        return

    def get_predicted_path(self, time_steps):

        # TODO: predict the path using ground truth state
        return