""" This is a simple test to verify CARLA setup on your local machine. """

__author__ = "Quentin Cheng"
__email__ = "hantingc@andrew.cmu.edu"

import glob
import os
import sys
import time
import ipdb
import numpy as np
import matplotlib.pyplot as plt

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
    get_lane_marker_linestring_from_right_lane_road_and_lane_id,
    toSLVT,
    toXYVT,
)
from lattice_generator import (
    LatticeGenerator,
    Road,
    Actions,
    Constraints,
    TerminationConditions,
    State,
    CostCalculator,
)
from agents.navigation.agent import Agent
from agents.navigation.local_planner_behavior import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.types_behavior import (
    Cautious,
    Aggressive,
    Normal,
    VeryAggressive,
)

from agents.tools.misc import positive

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
        self.world_snapshot = self.world.get_snapshot()
        self.time_step_count = 0
        self.time_step = 0.025  # Seconds. Have to fully divide 1
        self.curr_time = 0
        self.subject_path_time_res = 0.5
        self.warmup_time = 3
        # 0. Reset Scene
        initialize(self.world, self.client, self.time_step)

        self.lane_marker_linestring = (
            get_lane_marker_linestring_from_right_lane_road_and_lane_id(
                self.world, 15, -6
            )
        )

        # 1. Spawn vehicles
        self.subject_behavior = "very_aggressive"
        (
            self.ego_vehicle,
            self.subject_vehicle,
            self.current_lane_waypoints,
            self.subject_agent,
        ) = setup_scenario(
            self.world,
            self.client,
            synchronous_master=True,
            subject_behavior=self.subject_behavior,
        )

        # 2. Get the path follower object
        self.path_follower = PathFollower(self.world, self.ego_vehicle, self.time_step)

        # 3. Coarse Trajectory Genertion
        road = Road()
        actions = Actions()
        constraints = Constraints()
        termination_conditions = TerminationConditions(max_time=15, max_position_x=180)
        start_state = State(
            [self.ego_vehicle.get_location().x, self.ego_vehicle.get_location().y], 0, 0
        )
        cost_calculator = CostCalculator(
            subject_path_time_res=self.subject_path_time_res
        )
        self.latticeGenerator = LatticeGenerator(
            road=road,
            actions=actions,
            constraints=constraints,
            cost_calculator=cost_calculator,
            termination_conditions=termination_conditions,
            start_state=start_state,
            ego=self.ego_vehicle,
            subject=self.subject_vehicle,
        )

        # 3. Get the controller object
        args_lateral = {"K_P": 1.0, "K_D": 0.0, "K_I": 0.0, "dt": self.time_step}
        args_longitudinal = {
            "K_P": 1.0,
            "K_D": 0.00,
            "K_I": 0.00,
            "dt": self.time_step,
        }
        self.controller = VehiclePIDController(
            self.ego_vehicle, args_lateral, args_longitudinal
        )

        # 4. Placeholder for the concatenated trajectory
        self.traj_to_track = []
        self.has_lane_change_happend = 0
        self.is_lane_change_happening = 0
        self.planned_path = None
        self.next_timestep = None
        self.latest_tracked_speed_ego = get_speed(self.ego_vehicle)

        # self.regenerate_traj_flag = False

        if self.subject_behavior == "manual":
            self.path_predictor = PathPredictor(
                self.world, self.subject_vehicle, self.subject_path_time_res
            )
        else:
            self.path_predictor = PathPredictor(
                self.world, self.subject_agent.vehicle, self.subject_path_time_res
            )
        self.subject_traj = []
        self.ego_traj = []

    def plot_traj(self):

        fig, ax1 = plt.subplots(1, 1, figsize=(16, 6))
        vel = [state.speed for state in self.subject_traj]
        t = [self.time_step * i for i in range(len(vel))]
        ax1.plot(t, vel, label="Subject")
        vel = [state.speed for state in self.ego_traj]
        t = [self.time_step * i for i in range(len(vel))]
        ax1.plot(t, vel, label="Ego")
        ax1.set_xlabel("Time(s)")
        ax1.set_ylabel("Velocity(m/sec)")
        ax1.set_title("Velocity Profile")
        ax1.legend()

        fig.suptitle(
            "Velocity Profile for " + self.subject_behavior + " behavior", fontsize=20
        )

        if not os.path.exists("./results"):
            os.makedirs("./results")
        fig.savefig("./results/" + self.subject_behavior + str(time.time()) + ".png")

        np.save(vel, "./results/")
        plt.close(fig)

    def loop(self):

        # update_spectator(self.world, self.ego_vehicle)  # TODO: Spectator is buggy

        # 1. Get state estimation
        ego_transform = self.ego_vehicle.get_transform()

        if self.latest_tracked_speed_ego is None:
            ego_speed = get_speed(self.ego_vehicle)
        else:
            ego_speed = self.latest_tracked_speed_ego

        ego_state = State(
            [ego_transform.location.x, ego_transform.location.y],
            ego_speed,
            self.curr_time,
        )
        ego_state_slvt = toSLVT(self.lane_marker_linestring, ego_state)
        ego_state_pose = PoseTemp(
            ego_transform.location.x,
            ego_transform.location.y,
            ego_transform.rotation.yaw * np.pi / 180,
            ego_speed,
        )

        subject_transform = self.subject_vehicle.get_transform()

        subject_state = State(
            [subject_transform.location.x, subject_transform.location.y],
            get_speed(self.subject_vehicle),
            self.curr_time,
        )
        subject_state_slvt = toSLVT(self.lane_marker_linestring, subject_state)

        self.subject_traj.append(subject_state)
        self.ego_traj.append(ego_state)

        print("##############")
        print(subject_state_slvt)
        print(ego_state_slvt)
        print("##############")

        if self.subject_behavior != "manual":
            # 5. Predict the path of the suject agent
            if len(self.subject_agent.get_local_planner().waypoints_queue) != 0:
                self.subject_vehicle.apply_control(self.subject_agent.run_step())
            else:
                control = carla.VehicleControl()
                control.steer = 0.0
                control.throttle = 0.0
                control.brake = 1.0
                control.hand_brake = False
                control.manual_gear_shift = False
                self.subject_vehicle.apply_control(control)

        subject_path_slvt = []
        steps = 100
        subject_path = self.path_predictor.get_predicted_path(
            steps, subject_state_slvt.time
        )
        subject_path_slvt = [
            toSLVT(self.lane_marker_linestring, elem) for elem in subject_path
        ]
        # print([(pt.position[0], pt.speed, pt.time) for pt in subject_path_slvt])
        for i in range(len(subject_path)):
            self.world.debug.draw_string(
                carla.Location(
                    x=subject_path[i].position[0],
                    y=subject_path[i].position[1],
                    z=0,
                ),
                "X",
                draw_shadow=False,
                color=carla.Color(r=0, g=0, b=255),
            )

        # 2. Get next plan to track

        while (
            self.world.get_snapshot().timestamp.elapsed_seconds < 15
            and self.subject_behavior == "manual"
        ):
            # print("Time: ", self.world_snapshot.timestamp.elapsed_seconds)
            control_signal = self.controller.run_step(ego_speed, ego_state_pose)
            self.ego_vehicle.set_transform(
                carla.Transform(
                    location=carla.Location(ego_state_pose.x, ego_state_pose.y, 0),
                    rotation=carla.Rotation(yaw=ego_state_pose.theta * 180 / np.pi),
                )
            )
            # print("ego_speed: ", ego_speed)
            self.world.tick()

        if self.time_step_count % 10 == 0 and self.is_lane_change_happening == 0:

            self.time_step_count = 0

            # 6. Send current state to coarse path prediction module
            (
                full_lattice,
                state_2_cost,
                reverse_lattice,
                goal_state,
            ) = self.latticeGenerator.generate_full_lattice(
                ego_state_slvt, subject_path_slvt, self.has_lane_change_happend
            )

            # print(reverse_lattice)

            # 7. Get Best Goal States

            # 8. Backtrack to get best path

            # random.shuffle(goal_states)
            # example_goal_state_tuple = goal_states[0]

            example_start_state_tuple = (
                ego_state_slvt.position[0],
                ego_state_slvt.position[1],
                ego_state_slvt.speed,
                ego_state_slvt.time,
                self.has_lane_change_happend,
            )
            if goal_state is None:
                print(full_lattice)
                print(state_2_cost)

            if goal_state is not None:
                print(
                    "Goal State:",
                    goal_state,
                    "Cost:",
                    state_2_cost[goal_state],
                    "Ego State:",
                    ego_state_slvt.position[0],
                )

            (
                planned_path,
                planned_action_sequence,
            ) = self.latticeGenerator.backtrack_from_state(
                reverse_lattice,
                state_2_cost,
                goal_state,
                example_start_state_tuple,
            )
            self.planned_path = planned_path
            # Overwrite Trajectory for testing
            # planned_action_sequence = [0, 3, 0, 0, 0]

            print(planned_action_sequence)
            print(planned_path)
            print("Subject Path : --------------------")
            for elem in subject_path_slvt:
                print(elem)
            print("-----------------------------------")

            # if self.regenerate_traj_flag == False:
            (
                self.traj_to_track,
                self.lane_change_flags,
            ) = self.path_follower.get_trajectory(
                max(7.33 * 3.6, ego_state.speed * 3.6),
                get_ego_waypoint(self.world, self.ego_vehicle),
                planned_action_sequence,
            )
            # print([(pose.x, pose.y) for pose in self.traj_to_track])
            # self.regenerate_traj_flag = True

        # # 3. Find next pose to track
        # pose_to_track, next_index = self.path_follower.findNextLanePose(
        #     eog_state_pose, self.traj_to_track
        # )
        next_index = self.time_step_count
        self.next_timestep = next_index
        # print(next_index, "IDX")
        pose_to_track = self.traj_to_track[next_index]
        # print(self.planned_path)
        # print(
        #     (pose_to_track.x, pose_to_track.y, pose_to_track.speed),
        #     (ego_state.position[0], ego_state.position[1], ego_state.speed * 3.6),
        #     "TMP",
        # )
        # print(
        #     ego_state_slvt.position[0],
        #     ego_state_slvt.position[1],
        #     ego_state_slvt.time,
        #     "POS",
        # )
        # print(
        #     "-------------------------------------------------------------------------------"
        # )
        # print(
        #     "-------------------------------------------------------------------------------"
        # )

        next_flag = self.lane_change_flags[next_index]
        if next_flag == 1:
            self.has_lane_change_happend = 1
            self.is_lane_change_happening = 1
        else:
            self.is_lane_change_happening = 0

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
                life_time=0.25,
            )

        if pose_to_track is not None:
            # print(
            #     "Speed Tracking:",
            #     pose_to_track.speed,
            #     "Speed current:",
            #     ego_state.speed,
            #     "Plan progress:",
            #     float(next_index) / len(self.traj_to_track),
            # )
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
            # self.ego_vehicle.apply_control(control_signal)
            self.ego_vehicle.set_transform(
                carla.Transform(
                    location=carla.Location(pose_to_track.x, pose_to_track.y, 0),
                    rotation=carla.Rotation(yaw=pose_to_track.theta * 180 / np.pi),
                )
            )
            self.latest_tracked_speed_ego = pose_to_track.speed / 3.6

        # 7. TODO: check for collision at every time step

        # 8. Tick
        self.time_step_count += 1
        self.curr_time = self.time_step_count * self.time_step
        # print("Time: ", self.curr_time)
        self.world.tick()

    def play(self):

        try:
            while True:
                self.loop()

        except Exception as e:
            # print(e)
            self.plot_traj()
            # for a in self.world.get_actors().filter("vehicle*"):
            #     if a.is_alive:
            #         a.destroy()
            # reset_settings(self.world)


if __name__ == "__main__":

    manager = scenario_manager()
    manager.play()