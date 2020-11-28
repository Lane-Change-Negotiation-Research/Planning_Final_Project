#!/usr/bin/env python

""" This is a simple test to verify CARLA setup on your local machine. """

__author__ = "Quentin Cheng"
__email__ = "hantingc@andrew.cmu.edu"

import glob
import os
import sys
import time
import numpy as np
from util import (
    change_to_Town06,
    get_speed,
    draw_road_lane,
    waypoint_debug,
    setup_scenario,
    initialize,
    reset_settings,
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
from trajectory_generator import TrajGenerator, PoseTemp


class scenario_manager:
    def __init__(self):

        self.client = carla.Client("127.0.0.1", 2000)
        self.client.set_timeout(20.0)

        change_to_Town06(self.client)
        self.world = self.client.get_world()

        # 0. Reset Scene
        initialize(self.world, self.client)

        # 1. Spawn vehicles
        (
            self.ego_vehicle,
            self.subject_vehicle,
            self.current_lane_waypoints,
        ) = setup_scenario(self.world, self.client, synchronous_master=True)

        # 2. #TODO: Create behavior control/ behavior generation capability for subject vehicle(s).

        # 3. Coarse Trajectory Genertion #TODO: Wait for coarse trajectory generation capabilities to be built.

        # 2. Get the trajectory generator object #TODO: Wait for trajectory generation functionality to be built.
        self.trajectory_generator = TrajGenerator(self.current_lane_waypoints)

        # 3. Get the controller object
        self.controller = VehiclePIDController(self.ego_vehicle)
        self.trigger = False

    def loop(self):

        # 1. Get state estimation
        ego_transform = self.ego_vehicle.get_transform()
        ego_transform.rotation.yaw = ego_transform.rotation.yaw * np.pi / 180
        ego_speed = get_speed(self.ego_vehicle)

        subject_transform = self.subject_vehicle.get_transform()
        subject_transform.rotation.yaw = ego_transform.rotation.yaw * np.pi / 180
        subject_speed = get_speed(self.subject_vehicle)

        # 2. Get next plan to track #TODO: Wait for trajectory generation functionality to be built.
        if self.trigger == False:
            plan = self.trajectory_generator.laneChangeTraj(
                ego_transform, ego_speed, first_call=True
            )
            self.trigger = True
        else:
            plan = self.trajectory_generator.laneChangeTraj(
                ego_transform, ego_speed, first_call=False
            )

        # 3. Get control signal based on requested location + speed
        future_poses = plan["future_poses"]

        gamma = 1
        for tracking_pose in future_poses:
            gamma = gamma * 0.9
            self.world.debug.draw_string(
                Location(x=tracking_pose.x, y=tracking_pose.y, z=1),
                "O",
                draw_shadow=False,
                color=carla.Color(r=0, g=int(255 * gamma), b=0),
                life_time=1,
            )

        tracking_pose = plan["tracking_pose"]

        if tracking_pose is not None:
            print(
                "Speed Tracking:",
                tracking_pose.speed,
                "Speed current:",
                ego_speed,
                "Action progress:",
                plan["action_progress"],
                "End of action:",
                plan["end_of_action"],
            )
            self.world.debug.draw_string(
                Location(x=tracking_pose.x, y=tracking_pose.y, z=1),
                "O",
                draw_shadow=False,
                color=carla.Color(r=0, g=0, b=255),
                life_time=1,
            )

            control_signal = self.controller.run_step(
                tracking_pose.speed, tracking_pose
            )

            # 4. Apply control signal on ego-vehicle (actuation)
            self.ego_vehicle.apply_control(control_signal)

        # 5. Send current state to coarse path prediction module

        # 6. Tick
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
