# Miscellaneous utility functions

import math
import numpy as np
import glob
import os
import sys
import time
from typing import List
import time
import random

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
from carla import Location, Rotation, Transform


def change_to_Town06():
    client = carla.Client("127.0.0.1", 2000)
    client.set_timeout(10.0)
    world = client.load_world("Town06")
    # world = client.get_world()
    weather = world.get_weather()
    weather.cloudiness = 0
    weather.sun_altitude_angle = 45
    weather.precipitation_deposits = 0
    world.set_weather(weather)
    print("\ndone.")


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.
        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


def draw_road_lane(world, road_id, lane_id=None, color=[0, 255, 0]):
    all_waypoints = world.get_map().generate_waypoints(1)

    for wp in all_waypoints:

        if lane_id == None and wp.road_id == road_id:
            draw_waypoints(world, [wp], 3, color)
        elif wp.road_id == road_id and wp.lane_id == lane_id:
            draw_waypoints(world, [wp], 3, color)


def draw_waypoints(
    world: carla.libcarla.World,
    waypoints: List[carla.libcarla.Waypoint],
    life_time: float = 10.0,  # Seconds
    color: List[int] = [0, 255, 0],
    text="O",
) -> None:
    """
    Draws a list of waypoints in the given CARLA world.
    """
    for waypoint in waypoints:
        world.debug.draw_string(
            waypoint.transform.location,
            text,
            draw_shadow=False,
            color=carla.Color(r=color[0], g=color[1], b=color[2]),
            life_time=life_time,
            persistent_lines=True,
        )


def waypoint_debug(world):

    all_waypoints = world.get_map().generate_waypoints(3)

    for wp in all_waypoints:
        road_id = wp.road_id
        lane_id = wp.lane_id
        wp_loc = wp.transform.location
        letter = str(road_id) + "," + str(lane_id)
        world.debug.draw_string(
            wp_loc,
            letter,
            draw_shadow=False,
            color=carla.Color(r=0, g=255, b=0),
            life_time=20,
        )
    time.sleep(4)


def filter_waypoints(waypoints, road_id, lane_id=None):

    filtered_waypoints = []
    for wp in waypoints:

        if lane_id == None and wp.road_id == road_id:
            filtered_waypoints.append(wp)
        elif wp.road_id == road_id and wp.lane_id == lane_id:
            filtered_waypoints.append(wp)

    return filtered_waypoints


def spawn_vehicles(world):

    for a in world.get_actors().filter("vehicle*"):
        if a.is_alive:
            a.destroy()

    vehicles_list = []

    blueprints = world.get_blueprint_library()
    blueprints = [x for x in blueprints if x.id.endswith("model3")]

    all_waypoints = world.get_map().generate_waypoints(3)
    waypoint_list_lane_sub = filter_waypoints(all_waypoints, 15, -5)
    waypoint_list_lane_ego = filter_waypoints(all_waypoints, 15, -6)

    sub_spawn_point = waypoint_list_lane_sub[1].transform
    sub_spawn_point = carla.Transform(
        Location(x=sub_spawn_point.location.x, y=sub_spawn_point.location.y, z=0.5),
        Rotation(yaw=sub_spawn_point.rotation.yaw),
    )
    ego_spawn_point = waypoint_list_lane_ego[1].transform
    ego_spawn_point = carla.Transform(
        Location(x=ego_spawn_point.location.x, y=ego_spawn_point.location.y, z=0.5),
        Rotation(yaw=ego_spawn_point.rotation.yaw),
    )

    # --------------
    # Spawn vehicles
    # --------------
    blueprint = random.choice(blueprints)
    color = "0,0,255"
    blueprint.set_attribute("color", color)
    blueprint.set_attribute("role_name", "autopilot")

    # subject vehicle
    subject = world.spawn_actor(blueprint, sub_spawn_point)
    vehicles_list.append(subject)

    # ego vehicle
    color = "255,0,0"
    blueprint.set_attribute("color", color)
    ego = world.spawn_actor(blueprint, ego_spawn_point)
    vehicles_list.append(ego)
    world.tick()

    print("Spawned %d vehicles." % (len(vehicles_list)))
    return ego, subject
