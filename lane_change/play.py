#!/usr/bin/env python

""" This is a simple test to verify CARLA setup on your local machine. """

__author__ = "Quentin Cheng"
__email__ = "hantingc@andrew.cmu.edu"

import glob
import os
import sys
import time
from util import (
    change_to_Town06,
    get_speed,
    draw_road_lane,
    waypoint_debug,
    spawn_vehicles,
)

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
from trajectory_generator import TrajGenerator


class scenario_manager:
    def __init__(self):

        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(20.0)
        self.world = self.client.get_world()

        # 1. Spawn vehicles
        self.ego_vehicle, self.subject_vehicle = spawn_vehicles(self.world)

        # 2. #TODO: Create behavior control/ behavior generation capability for subject vehicle(s).

        # 3. Coarse Trajectory Genertion #TODO: Wait for coarse trajectory generation capabilities to be built.

        # 2. Get the trajectory generator object #TODO: Wait for trajectory generation functionality to be built.
        self.trajectory_generator = TrajGenerator()

        # 3. Get the controller object
        self.controller = VehiclePIDController(self.ego_vehicle)

    def loop(self):

        # 1. Get state estimation
        ego_transform = self.ego_vehicle.get_transform()
        ego_speed = get_speed(self.ego_vehicle)

        subject_transform = self.subject_vehicle.get_transform()
        subject_speed = get_speed(self.subject_vehicle)

        # 2. Get next waypoint to track #TODO: Wait for trajectory generation functionality to be built.
        next_location, next_speed = self.trajectory_generator.get_next(ego_transform)

        # 3. Get control signal based on requested location + speed
        control_signal = self.controller.run_step(next_location, next_speed)

        # 4. Apply control signal on ego-vehicle (actuation)
        self.ego_vehicle.apply_control(control_signal)

        # 5. Send current state to coarse path prediction module

    def play(self):

        try:

            while true:
                self.loop()

        except:

            for a in self.world.get_actors().filter("vehicle*"):
                if a.is_alive:
                    a.destroy()