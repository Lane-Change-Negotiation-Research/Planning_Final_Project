""" Collision Checker takes the lattice graphs generated by Lattice Generator and modifies it 
based on the predicted subject path from Subject Predictor and the CARLA road environment. """
from carla import *
import numpy as np
import math

class CollisionChecker:
    def __init__(self, time=1):

        # self.lattice_graph = None
        self.subject_path = None
        self.time_buffer = time
    
    def predict_collision(self, ego_size, subject_size, ego_state, subject_path):
        collide = False
        for sub_state in subject_path:
            collide = self.check_collision(ego_size, subject_size, ego_state, sub_state)
        
            if(collide):
                break

        return collide

    def check_collision(self, ego_size, sub_size, ego_state, sub_state):
        if ego_state.time - sub_state.time > self.time_buffer:
            return False

        ego_rad = math.sqrt(pow(ego_size[0], 2) + pow(ego_size[1], 2))
        sub_rad = math.sqrt(pow(sub_size[0], 2) + pow(sub_size[1], 2))
        center_dist = math.sqrt(pow(ego_state.position[0] - sub_state.position[0], 2) + pow(ego_state.position[1] - sub_state.position[1], 2))
        return center_dist <= ego_rad + sub_rad

    # def update_lattice(self, lattice_graph):
    #     self.lattice_graph = lattice_graph

    def update_subject_path(self, subject_path):
        self.subject_path = subject_path

    def assign_cost(self):
        pass
