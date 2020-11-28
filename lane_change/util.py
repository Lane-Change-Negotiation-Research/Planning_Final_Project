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
import re

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
from trajectory_generator import PoseTemp


def change_to_Town06(client):

    world = client.load_world("Town06")

    world.set_weather(find_weather_presets()[0][0])

    print("Map 06 has been loaded into the CARLA world...")


def find_weather_presets():
    rgx = re.compile(".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)")
    name = lambda x: " ".join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match("[A-Z].+", x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


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


def initialize(world, client):
    actors = world.get_actors()
    client.apply_batch(
        [carla.command.DestroyActor(x) for x in actors if "vehicle" in x.type_id]
    )
    for a in actors.filter("vehicle*"):
        if a.is_alive:
            a.destroy()
    world.tick()
    settings = world.get_settings()
    settings.synchronous_mode = True  # Enables synchronous mode
    settings.fixed_delta_seconds = 0.05
    world.apply_settings(settings)

    print("Scene init done!")


def reset_settings(world):

    settings = world.get_settings()
    settings.synchronous_mode = False  # Enables synchronous mode
    settings.fixed_delta_seconds = 0
    world.apply_settings(settings)


def setup_scenario(world, client, synchronous_master=False):

    # @todo cannot import these directly.
    SpawnActor = carla.command.SpawnActor
    SetAutopilot = carla.command.SetAutopilot
    FutureActor = carla.command.FutureActor

    batch = []
    ego_batch = []

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

    # Subject Vehicle Details
    blueprint = random.choice(blueprints)
    color = "0,0,255"
    blueprint.set_attribute("color", color)
    blueprint.set_attribute("role_name", "autopilot")
    batch.append(SpawnActor(blueprint, sub_spawn_point))

    # Ego Vehicle Details
    color = "255,0,0"
    blueprint.set_attribute("color", color)
    blueprint.set_attribute("role_name", "ego")
    ego_batch.append(
        SpawnActor(blueprint, ego_spawn_point).then(SetAutopilot(FutureActor, True))
    )

    # Spawn
    ego_vehicle_id = None
    for response in client.apply_batch_sync(ego_batch, synchronous_master):
        if response.error:
            print("Response Error while applying ego batch!")
        else:
            # self.vehicles_list.append(response.actor_id)
            ego_vehicle_id = response.actor_id
    print("Ego vehicle id: ------------------------------", ego_vehicle_id)
    for response in client.apply_batch_sync(batch, synchronous_master):
        if response.error:
            print("Response Error while applying batch!")
        else:
            vehicles_list.append(response.actor_id)

    subject_vehicle = world.get_actors(vehicles_list)[
        0
    ]  # 0 because only 1 vehicle being spawned
    ego_vehicle = world.get_actors([ego_vehicle_id])[0]

    print("Warm start initiated...")
    warm_start_curr = 0
    while warm_start_curr < 5:
        warm_start_curr += 0.05
        if synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()

    client.apply_batch_sync([SetAutopilot(ego_vehicle, False)], synchronous_master)
    print("Warm start finished...")

    ## Get current lane waypoints
    ego_vehicle_location = ego_vehicle.get_location()
    nearest_waypoint = world.get_map().get_waypoint(
        ego_vehicle_location, project_to_road=True
    )
    current_lane_waypoints = nearest_waypoint.next_until_lane_end(1)

    for wp in current_lane_waypoints:
        world.debug.draw_string(
            Location(x=wp.transform.location.x, y=wp.transform.location.y, z=1),
            "O",
            draw_shadow=False,
            color=carla.Color(r=255, g=0, b=0),
            life_time=10,
        )

    return (ego_vehicle, subject_vehicle, current_lane_waypoints)
