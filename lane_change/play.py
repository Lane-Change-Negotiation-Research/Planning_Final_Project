""" This is a simple test to verify CARLA setup on your local machine. """

__author__ = "Quentin Cheng"
__email__ = "hantingc@andrew.cmu.edu"

import glob
import os
import sys
import time
import ipdb
import numpy as np
from util import (
    change_to_Town06,
    get_speed,
    draw_road_lane,
    waypoint_debug,
    setup_scenario,
    initialize,
    reset_settings,
    get_ego_waypoint,
    update_spectator,
)
from lattice_generator import LatticeGenerator, Road, Actions, Constraints, TerminationConditions, State, CostCalculator
from agents.navigation.agent import Agent
from agents.navigation.local_planner_behavior import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.types_behavior import Cautious, Aggressive, Normal

from agents.tools.misc import get_speed, positive

try:
    sys.path.append(
        glob.glob(
            "../carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

import carla
from carla import *
from PID_controller import VehiclePIDController

import argparse
import logging
import random
from enum import Enum

from PID_controller import VehiclePIDController
from trajectory_generator import TrajGenerator, PoseTemp, PathFollower
from path_predictor import PathPredictor


class scenario_manager:
    def __init__(self):

        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(20.0)

        change_to_Town06(self.client)
        self.world = self.client.get_world()
        self.time_step_count = 0
        self.time_step = 0.03
        self.curr_time = 0
        self.subject_path_time_res = 0.5

        # 0. Reset Scene
        initialize(self.world, self.client, self.time_step)

        # 1. Spawn vehicles
        self.subject_behavior = "aggressive"
        (
            self.ego_vehicle,
            self.subject_vehicle,
            self.current_lane_waypoints,
            self.subject_agent
        ) = setup_scenario(self.world, self.client, synchronous_master=True, subject_behavior=self.subject_behavior)
        update_spectator(self.world, self.ego_vehicle) #TODO: Spectator is buggy

        # 2. Get the path follower object
        self.path_follower = PathFollower(self.world, self.ego_vehicle)

        # 3. Coarse Trajectory Genertion #TODO: Wait for coarse trajectory generation capabilities to be built.
        road = Road()
        actions = Actions()
        constraints = Constraints()
        termination_conditions = TerminationConditions()
        start_state = State([self.ego_vehicle.get_location().x,self.ego_vehicle.get_location().y], 0, 0)
        cost_calculator = CostCalculator(subject_path_time_res=self.subject_path_time_res)
        self.latticeGenerator = LatticeGenerator(road = road,\
                                    actions = actions,\
                                    constraints= constraints,\
                                    cost_calculator = cost_calculator,\
                                    termination_conditions=termination_conditions,\
                                    start_state=start_state,\
                                    ego=self.ego_vehicle,
                                    subject=self.subject_vehicle
                                    )


        # 3. Get the controller object
        args_lateral = {'K_P': 0.75,
            'K_D': 0.02,
            'K_I': 0.4,
            'dt': self.time_step}
        args_longitudinal = {'K_P': 1.0,
            'K_D': 0.024,
            'K_I': 0.032, 
            'dt': self.time_step}
        self.controller = VehiclePIDController(self.ego_vehicle, args_lateral, args_longitudinal)

        # 4. Placeholder for the concatenated trajectory
        self.traj_to_track = []

        self.regenerate_traj_flag = False

        # WIP: add path prediction module
        self.path_predictor = PathPredictor(self.world, self.subject_agent, self.subject_path_time_res)


    def loop(self):

        # 1. Get state estimation
        ego_transform = self.ego_vehicle.get_transform()

        ego_state = State(
            [ego_transform.location.x, ego_transform.location.y],
            get_speed(self.ego_vehicle),
            self.curr_time)

        subject_transform = self.subject_vehicle.get_transform()

        subject_state = State(
            [subject_transform.location.x, subject_transform.location.y],
            get_speed(self.subject_vehicle),
            self.curr_time)

        # 2. Get next plan to track.
        if self.regenerate_traj_flag == False:
            self.traj_to_track = self.path_follower.get_trajectory(
                ego_state.speed, get_ego_waypoint(self.world, self.ego_vehicle)
            )
            self.regenerate_traj_flag = True

        # 3. Find next pose to track
        # pose_to_track, next_index = self.path_follower.findNextLanePose(
        #     ego_state, self.traj_to_track
        # )
        next_index = self.time_step_count
        pose_to_track = self.traj_to_track[next_index]

        # 3. Get control signal based on requested location + speed
        future_poses = self.traj_to_track[next_index:]

        gamma = 1
        for future_pose in future_poses:
            gamma = gamma * 0.98
            self.world.debug.draw_string(
                Location(x=future_pose.x, y=future_pose.y, z=1),
                "O",
                draw_shadow=False,
                color=carla.Color(r=0, g=int(255 * gamma), b=0),
                life_time=1,
            )

        if pose_to_track is not None:
            print(
                "Speed Tracking:",
                pose_to_track.speed,
                "Speed current:",
                ego_state.speed,
                "Plan progress:",
                float(next_index) / len(self.traj_to_track),
            )
            self.world.debug.draw_string(
                Location(x=pose_to_track.x, y=pose_to_track.y, z=1),
                "O",
                draw_shadow=False,
                color=carla.Color(r=0, g=0, b=255),
                life_time=1,
            )

            control_signal = self.controller.run_step(
                pose_to_track.speed, pose_to_track
            )

            # 4. Apply control signal on ego-vehicle and subject vehicle(actuation)
            self.ego_vehicle.apply_control(control_signal)


                
        if(self.time_step_count%25):
            # 5. Predict the path of the suject agent
            subject_path = []
            if(len(self.subject_agent.get_local_planner().waypoints_queue) != 0):
                self.subject_vehicle.apply_control(self.subject_agent.run_step())
                steps = 10
                subject_path = self.path_predictor.get_predicted_path(steps)
                for i in range(len(subject_path)):
                    self.world.debug.draw_string(
                        carla.Location(x=subject_path[i].position[0], y=subject_path[i].position[1], z=0),
                        "X",
                        draw_shadow=False,
                        color=carla.Color(r=0, g=0, b=255)
                        
                    )
            else:
                control = carla.VehicleControl()
                control.steer = 0.0
                control.throttle = 0.0
                control.brake = 1.0
                control.hand_brake = False
                control.manual_gear_shift = False
                self.subject_vehicle.apply_control(control)            

            # 6. Send current state to coarse path prediction module
            full_lattice, state_2_cost = self.latticeGenerator.generate_full_lattice(ego_state, subject_path)        

        #7. TODO: check for collision at every time step

        #8. Tick
        self.time_step_count += 1
        self.curr_time = self.time_step_count*self.time_step 
        print("Time: ", self.curr_time)
        self.world.tick()

    def play(self):

        # try:
        while True:
            self.loop()

        # except:

        #     for a in self.world.get_actors().filter("vehicle*"):
        #         if a.is_alive:
        #             a.destroy()
        #     reset_settings(self.world)


if __name__ == "__main__":

    manager = scenario_manager()
    manager.play()