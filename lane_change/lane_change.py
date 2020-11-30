#!/usr/bin/env python

""" Working progress of lane change scenario. """

__author__ = "Quentin Cheng"
__email__ = "hantingc@andrew.cmu.edu"

import glob
import os
import sys
import time
import numpy as np
from util import *

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
from carla import *
from PID_controller import VehiclePIDController

import argparse
import logging
import random
from enum import Enum


def get_state_information(ego_vehicle=None):

    if(ego_vehicle == None):
        print("No ego vehicle specified..")
        return None
    else:
        # Get ego vehicle location and nearest waypoint for reference.
        ego_vehicle_location = ego_vehicle.get_location()
        nearest_waypoint = world.get_map().get_waypoint(
            ego_vehicle_location, project_to_road=True)

        ego_speed = np.sqrt(ego_vehicle.get_velocity(
        ).x**2 + ego_vehicle.get_velocity().y**2 + ego_vehicle.get_velocity().z**2) * 3.6

        current_lane_waypoints = get_next_waypoints(
            nearest_waypoint, ego_speed, k=2)[::-1]
        # left_lane_waypoints =  self.get_next_waypoints(nearest_waypoint.get_left_lane(), ego_speed, k=300)[::-1] #+
        right_lane_waypoints = get_next_waypoints(
            nearest_waypoint.get_right_lane(), ego_speed, k=2)[::-1]  # +

    return ego_speed, nearest_waypoint, current_lane_waypoints, right_lane_waypoints


def get_next_waypoints(last_waypoint, ego_speed, rev=False, k=100):

    if(last_waypoint == None):
        return []

    sampling_radius = 1  # ego_speed * 1 / 3.6
    full_waypoints = []
    for i in range(k):
        if(rev == False):
            next_waypoints = last_waypoint.next(sampling_radius)
        else:
            next_waypoints = last_waypoint.previous(sampling_radius)

        if len(next_waypoints) == 0:
            break
        else:
            # only one option available ==> lanefollowing
            next_waypoint = next_waypoints[0]

        full_waypoints.append(next_waypoint)
        # curr_waypoint = next_waypoints[-1]

        last_waypoint = next_waypoint

    return full_waypoints


def initialize(world):
    actors = world.get_actors()
    client.apply_batch([carla.command.DestroyActor(x)
                        for x in actors if 'vehicle' in x.type_id])
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.03
    world.apply_settings(settings)


def spawn_vehicles():

    vehicles_list = []

    blueprints = world.get_blueprint_library()
    blueprints = [x for x in blueprints if x.id.endswith('model3')]

    # spawn_points = world.get_map().get_spawn_points()
    sub_spawn_point = carla.Transform(
        Location(x=530, y=-13.6, z=0.3), Rotation(yaw=180))
    ego_spawn_point = carla.Transform(
        Location(x=535, y=-10.1, z=0.3), Rotation(yaw=180))

    # --------------
    # Spawn vehicles
    # --------------
    blueprint = random.choice(blueprints)
    color = '0,0,255'
    blueprint.set_attribute('color', color)
    blueprint.set_attribute('role_name', 'autopilot')

    # subject vehicle
    subject = world.spawn_actor(blueprint, sub_spawn_point)
    vehicles_list.append(subject)

    # ego vehicle
    color = '255,0,0'
    blueprint.set_attribute('color', color)
    ego = world.spawn_actor(blueprint, ego_spawn_point)
    vehicles_list.append(ego)
    world.tick()

    print('spawned %d vehicles.' % (len(vehicles_list)))
    return ego, subject


def main(args):
    pass


if __name__ == '__main__':

    argparser = argparse.ArgumentParser(
        description=__doc__)
    argparser.add_argument(
        '--host',
        metavar='H',
        default='127.0.0.1',
        help='IP of the host server (default: 127.0.0.1)')
    argparser.add_argument(
        '-p', '--port',
        metavar='P',
        default=2000,
        type=int,
        help='TCP port to listen to (default: 2000)')
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '--sync',
        action='store_true',
        default=True,
        help='Synchronous mode execution')
    args = argparser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(20.0)
    world = client.get_world()
    initialize(world)
    ego, subject = spawn_vehicles()

    # traffice manager settings
    traffic_manager = client.get_trafficmanager(args.tm_port)
    tm_port = traffic_manager.get_port()
    traffic_manager.global_percentage_speed_difference(0)
    subject.set_autopilot(True, tm_port)
    world.tick()

    # Instantiate Ego Vehicle Controller
    vehicle_controller = VehiclePIDController(
        ego, args_longitudinal={'K_P': 1.0, 'K_D': 0.0, 'K_I': 0.0})

    try:

        while True:
            # Get future trajectory
            ego_speed, nearest_waypoint, current_lane_waypoints, right_lane_waypoints = get_state_information(
                ego)
            # ego_speed = get_speed(ego)
            print("Driving at ", ego_speed, " m/s")
            # subject_speed, subject_waypoint, subject_current_lane_waypoints, _ = get_state_information(subject)

            # TODO: Use a lattice-planner here
            # Given a current position, generate motion primitives that
            # 1. Goes Straight, Constant Speed
            # 2. Goes Straight, Speed up
            # 3. Goes Straight, Slow down
            # 4. Lane Change Right

            # Predict Subject vehicle position in the next 3 time instances

            # Build map going 3 motion primitives forward that avoid subject vehicle and assign costs

            # Search for best

            # Return target_speed and waypoint to track
            target_speed = 50  # km/hr
            # counter = 0
            # if counter > 20:
            #     waypoint = right_lane_waypoints[counter]
            # else:
            #     waypoint = current_lane_waypoints[counter]
            # counter+=1
            waypoint = current_lane_waypoints[0]

            # Apply Contorl
            pose = waypoint.transform.location
            control = vehicle_controller.run_step(target_speed, pose)
            ego.apply_control(control)
            world.tick()

    finally:
        print("Destroying: ego")
        ego.destroy()
        print("Destroying: subject")
        subject.destroy()

        time.sleep(0.5)
