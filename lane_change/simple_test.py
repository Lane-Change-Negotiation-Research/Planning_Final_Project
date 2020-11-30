#!/usr/bin/env python

""" This is a simple test to verify CARLA setup on your local machine. """

__author__ = "Quentin Cheng"
__email__ = "hantingc@andrew.cmu.edu"

# from behavior_agent import BehaviorAgent
import glob
import os
import sys
import time
import numpy as np
from util import *
try:
    import pygame
    from pygame.locals import KMOD_CTRL
    from pygame.locals import K_ESCAPE
    from pygame.locals import K_q
except ImportError:
    raise RuntimeError('cannot import pygame, make sure pygame package is installed')
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


def initialize(world):
    actors = world.get_actors()
    client.apply_batch([carla.command.DestroyActor(x)
                        for x in actors if 'vehicle' in x.type_id])
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)


def spawn_vehicles():

    vehicles_list = []

    blueprints = world.get_blueprint_library()
    blueprints = [x for x in blueprints if x.id.endswith('model3')]

    # spawn_points = world.get_map().get_spawn_points()
    sub_spawn_point = carla.Transform(
        Location(x=20, y=45.3, z=0.3), Rotation(yaw=0))
    ego_spawn_point = carla.Transform(
        Location(x=15, y=41.8, z=0.3), Rotation(yaw=0))

    # --------------
    # Spawn vehicles
    # --------------
    blueprint = random.choice(blueprints)
    color = '0,0,255'
    blueprint.set_attribute('color', color)
    blueprint.set_attribute('role_name', 'autopilot')

    # subject vehicle
    subject = world.spawn_actor(blueprint, sub_spawn_point)
    # subject_agent = BehaviorAgent(subject, behavior='normal')
    # spawn_points = world.map.get_spawn_points()
    # random.shuffle(spawn_points)

    # if spawn_points[0].location != subject_agent.vehicle.get_location():
    #     destination = spawn_points[0].location
    # else:
    #     destination = spawn_points[1].location

    # subject_agent.set_destination(subject_agent.vehicle.get_location(), destination, clean=True)
    vehicles_list.append(subject)

    # ego vehicle
    color = '255,0,0'
    blueprint.set_attribute('color', color)
    ego = world.spawn_actor(blueprint, ego_spawn_point)
    vehicles_list.append(ego)
    world.tick()

    print('spawned %d vehicles.' % (len(vehicles_list)))
    return ego, subject


def control():
    control = ego.get_control()
    control.throttle = 0
    ego.apply_control(control)
    counter = 85
    clock = pygame.time.Clock()
    while True:
        clock.tick_busy_loop(60)
        cur_speed = get_speed(ego)
        print('Current Speed: ', cur_speed)
        world.tick()
        counter += 1
        if counter > 100:
            control.throttle = 1
        if counter > 200:
            control.steer = 0.25
        if counter > 209:
            control.steer = -0.265
        if counter > 218:
            control.steer = 0
        ego.apply_control(control)


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

    change_to_Town06()
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

    # import ipdb; ipdb.set_trace()
    try:
        control()

    finally:
        print("Destroying: ego")
        ego.destroy()
        print("Destroying: subject")
        subject.destroy()

        time.sleep(0.5)
