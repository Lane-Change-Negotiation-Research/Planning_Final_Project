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
    get_ego_waypoint,
    update_spectator,
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
from trajectory_generator import TrajGenerator, PoseTemp, PathFollower


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
        update_spectator(self.world, self.ego_vehicle) #TODO: Spectator is buggy

        # 2. #TODO: Create behavior control/ behavior generation capability for subject vehicle(s).

        # 3. Coarse Trajectory Genertion #TODO: Wait for coarse trajectory generation capabilities to be built.

        # 2. Get the path follower object
        self.path_follower = PathFollower(self.world, self.ego_vehicle)

        # 3. Get the controller object
        self.controller = VehiclePIDController(self.ego_vehicle)

        # 4. Placeholder for the concatenated trajectory
        self.traj_to_track = []

        self.regenerate_traj_flag = False

    def loop(self):

        # 1. Get state estimation
        ego_transform = self.ego_vehicle.get_transform()
        # ego_transform.rotation.yaw = ego_transform.rotation.yaw
        # ego_speed = get_speed(self.ego_vehicle)
        ego_pose = PoseTemp(
            x=ego_transform.location.x,
            y=ego_transform.location.y,
            theta=ego_transform.rotation.yaw * np.pi / 180,
            speed=get_speed(self.ego_vehicle),
        )

        subject_transform = self.subject_vehicle.get_transform()
        # subject_transform.rotation.yaw = ego_transform.rotation.yaw * np.pi / 180
        # subject_speed = get_speed(self.subject_vehicle)
        subject_pose = PoseTemp(
            x=subject_transform.location.x,
            y=subject_transform.location.y,
            theta=subject_transform.rotation.yaw * np.pi / 180,
            speed=get_speed(self.subject_vehicle),
        )

        # 2. Get next plan to track.
        if self.regenerate_traj_flag == False:
            self.traj_to_track = self.path_follower.get_trajectory(
                ego_pose.speed, get_ego_waypoint(self.world, self.ego_vehicle)
            )
            self.regenerate_traj_flag = True

        # 3. Find next pose to track
        pose_to_track, next_index = self.path_follower.findNextLanePose(
            ego_pose, self.traj_to_track
        )

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
                ego_pose.speed,
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

            # 4. Apply control signal on ego-vehicle (actuation)
            self.ego_vehicle.apply_control(control_signal)

        # 5. Send current state to coarse path prediction module

        # 6. Tick
        self.world.tick()

    def play(self):

        try:

            while True:
                self.loop()

        except:

            for a in self.world.get_actors().filter("vehicle*"):
                if a.is_alive:
                    a.destroy()
            reset_settings(self.world)


if __name__ == "__main__":

    manager = scenario_manager()
    manager.play()