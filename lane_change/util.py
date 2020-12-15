# Miscellaneous utility functions

import math
import numpy as np
import glob
import os
import sys
import time
from typing import List, Tuple
import time
import random
import re

try:
    sys.path.append(
        glob.glob(
            "/opt/CARLA_0.9.9.4/PythonAPI/carla/dist/carla-*%d.%d-%s.egg"
            % (
                sys.version_info.major,
                sys.version_info.minor,
                "win-amd64" if os.name == "nt" else "linux-x86_64",
            )
        )[0]
    )
except IndexError:
    pass

from agents.navigation.behavior_agent import BehaviorAgent
from agents.navigation.agent import Agent
from agents.navigation.local_planner_behavior import LocalPlanner, RoadOption
from agents.navigation.global_route_planner import GlobalRoutePlanner
from agents.navigation.global_route_planner_dao import GlobalRoutePlannerDAO
from agents.navigation.types_behavior import Cautious, Aggressive, Normal

from agents.tools.misc import get_speed, positive

import carla
from carla import Location, Rotation, Transform
from shapely.geometry import LineString, Point
from lattice_generator import State


def change_to_Town06(client):

    world = client.load_world("Town06")

    world.set_weather(find_weather_presets()[0][0])

    print("Map 06 has been loaded into the CARLA world...")


def find_weather_presets():
    rgx = re.compile(".+?(?:(?<=[a-z])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])|$)")
    name = lambda x: " ".join(m.group(0) for m in rgx.finditer(x))
    presets = [x for x in dir(carla.WeatherParameters) if re.match("[A-Z].+", x)]
    return [(getattr(carla.WeatherParameters, x), name(x)) for x in presets]


def get_ego_waypoint(world, ego_vehicle):

    ego_vehicle_location = ego_vehicle.get_location()
    nearest_waypoint = world.get_map().get_waypoint(
        ego_vehicle_location, project_to_road=True
    )

    return nearest_waypoint


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.
        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in m/s
    """
    vel = vehicle.get_velocity()

    return math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)


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


def setup_scenario(world, client, synchronous_master=False, subject_behavior="normal"):

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
    # waypoint_list_lane_sub = filter_waypoints(all_waypoints, 15, -3)
    waypoint_list_lane_ego = filter_waypoints(all_waypoints, 15, -6)

    sub_spawn_point = waypoint_list_lane_sub[1].transform
    # sub_spawn_point = waypoint_list_lane_sub[8].transform
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
    batch.append(
        SpawnActor(blueprint, sub_spawn_point).then(SetAutopilot(FutureActor, True))
    )

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
    # client.apply_batch_sync([SetAutopilot(subject_vehicle, False)], synchronous_master)

    ego_vehicle = world.get_actors([ego_vehicle_id])[0]

    # update_spectator(world, ego_vehicle)

    print("Warm start initiated...")
    warm_start_curr = 0
    while warm_start_curr < 3:
        warm_start_curr += world.get_settings().fixed_delta_seconds
        if synchronous_master:
            world.tick()
        else:
            world.wait_for_tick()

    client.apply_batch_sync([SetAutopilot(ego_vehicle, False)], synchronous_master)
    client.apply_batch_sync([SetAutopilot(subject_vehicle, False)], synchronous_master)

    subject_agent = BehaviorAgent(subject_vehicle, behavior=subject_behavior)
    destination = carla.Location(x=190.50791931152344, y=45.247249603271484, z=0.0)
    # destination_wp = world.get_map().get_waypoint(subject_vehicle.get_location()).next_until_lane_end(10)[-1]
    # destination_wp = destination_wp.next(40)[0]
    # destination = destination_wp.transform.location
    subject_agent.set_destination(
        subject_agent.vehicle.get_location(), destination, clean=True
    )

    subject_agent.update_information(world)

    print("Warm start finished...")

    ## Get current lane waypoints
    ego_vehicle_location = ego_vehicle.get_location()
    nearest_waypoint = world.get_map().get_waypoint(
        ego_vehicle_location, project_to_road=True
    )
    current_lane_waypoints = nearest_waypoint.next_until_lane_end(1)

    # for wp in current_lane_waypoints:
    #     world.debug.draw_string(
    #         Location(x=wp.transform.location.x, y=wp.transform.location.y, z=1),
    #         "O",
    #         draw_shadow=False,
    #         color=carla.Color(r=255, g=0, b=0),
    #         life_time=10,
    #     )

    return (ego_vehicle, subject_vehicle, current_lane_waypoints, subject_agent)


def initialize(world, client, time_step):
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
    settings.fixed_delta_seconds = time_step
    world.apply_settings(settings)

    print("Scene init done!")


def reset_settings(world):

    settings = world.get_settings()
    settings.synchronous_mode = False  # Enables synchronous mode
    settings.fixed_delta_seconds = 0
    world.apply_settings(settings)


def update_spectator(world, ego_vehicle):
    spectator = world.get_spectator()
    spectator_transform = ego_vehicle.get_transform()
    spectator_transform = carla.Transform(
        spectator_transform.location + carla.Location(x=-15.5, z=8.5),
        carla.Rotation(pitch=-10.0),
    )
    spectator.set_transform(spectator_transform)


def get_dummy_camera(world, ego_vehicle):

    bp_library = world.get_blueprint_library()
    camera_bp = bp_library.find("sensor.camera.rgb")
    camera_bp.set_attribute("image_size_x", "1")
    camera_bp.set_attribute("image_size_y", "1")

    transform = carla.Transform(
        carla.Location(x=-15.5, z=8.5), carla.Rotation(pitch=-10.0)
    )
    Attachment = carla.AttachmentType
    camera = world.spawn_actor(
        camera_bp, transform, attach_to=ego_vehicle, attachment_type=Attachment.Rigid
    )
    # camera.attributes["sensor_tick"] = 10
    # camera_parent_vehicle = ego_vehicle

    return camera


def getRightVector(rotation):
    cy = np.cos(np.radians(rotation.yaw))
    sy = np.sin(np.radians(rotation.yaw))
    cr = np.cos(np.radians(rotation.roll))
    sr = np.sin(np.radians(rotation.roll))
    cp = np.cos(np.radians(rotation.pitch))
    sp = np.sin(np.radians(rotation.pitch))
    return carla.Vector3D(cy * sp * sr - sy * cr, sy * sp * sr + cy * cr, -cp * sr)


def get_lane_marker_linestring_from_right_lane_road_and_lane_id(
    world, road_id, lane_id
):

    all_waypoints = world.get_map().generate_waypoints(3)

    filtered_waypoints = []
    for wp in all_waypoints:

        if wp.lane_id == lane_id and wp.road_id == road_id:
            filtered_waypoints.append(wp)

    first_wp = filtered_waypoints[0]

    next_wps = []
    for i in range(150):
        next_wps.append(first_wp.next(3)[0])
        first_wp = first_wp.next(3)[0]

    lane_marker_locations = []
    for wp in next_wps:

        # Rotate forward vector 90 degrees to get right vector
        # temp_vector = wp.transform.get_forward_vector()
        # x = temp_vector.x
        # temp_vector.x = temp_vector.y
        # temp_vector.y = -x
        right_vector = wp.lane_width / 2 * getRightVector(wp.transform.rotation)
        lane_marker_location = Location(
            x=wp.transform.location.x + right_vector.x,
            y=wp.transform.location.y - right_vector.y,
            z=wp.transform.location.z,
        )

        lane_marker_locations.append(lane_marker_location)

    for loc in lane_marker_locations:

        world.debug.draw_string(
            loc,
            "o",
            draw_shadow=False,
            color=carla.Color(r=0, g=255, b=0),
            life_time=10,
        )

    linestring = LineString([[loc.x, loc.y] for loc in lane_marker_locations])

    return linestring


def toSLVT(linestring, state):

    point = Point(state.position)
    s, l, _, _ = get_frenet_from_cartesian(linestring, point, 0)

    transformed_state = State([s, l], state.speed, state.time)
    return transformed_state


def toXYVT(linestring, state):

    x, y, _, _ = get_frenet_from_cartesian(linestring, state.position, 0)

    transformed_state = State([x, y], state.speed, state.time)
    return transformed_state


def get_frenet_from_cartesian(
    linestring: LineString,
    cartesian_point: Point,
    cartesian_heading: float = None,
    resolution: float = 0.01,
) -> Tuple[float, float, float, Point]:

    """
    Converts a point (x,y,heading) in the cartesian frame to it's frenet frame representation.
    :param linestring:
        A shapely 'LineString' (Polyline) that represents the centre line of the lane being used as
        the reference for the frenet frame.
    :param cartesian_point:
        A shapely 'Point' of the point in cartesian frame that has to be represented in the frenet frame.
    :param cartesian_heading:
        The heading associated with the cartesian point, in degrees.
    :param resolution:
        The resolution used to calculate the heading direction of local sections of the lane LineString.
    :returns:
        Tuple of (s, d, frenet_heading, projection of cartesian_point on linestring)
    """

    # Get s and d. d is not associated with a direction here.
    s = linestring.project(cartesian_point)
    d = linestring.distance(cartesian_point)

    # Find closest point (to the input point) on the lane LineString.
    closest_point_on_linestring = linestring.interpolate(s)

    # Find heading direction of the lane LineString by using a small section of the LineString,
    # around closest_point_on_linestring computed above.
    local_section_of_spline_start = linestring.interpolate(max(0, s - resolution))
    local_section_of_spline_end = linestring.interpolate(
        min(linestring.length, s + resolution)
    )
    local_section_heading_in_cartesian_coordinates = np.degrees(
        np.arctan2(
            local_section_of_spline_end.y - local_section_of_spline_start.y,
            local_section_of_spline_end.x - local_section_of_spline_start.x,
        )
    )

    # Find heading in frenet frame.
    heading_relative_to_local_section_heading = (
        cartesian_heading - local_section_heading_in_cartesian_coordinates
    )

    # Assign a direction (+ or -) to the distance d.
    heading_of_line_joining_input_points_and_its_closest_point_on_linestring = (
        np.degrees(
            np.arctan2(
                cartesian_point.y - closest_point_on_linestring.y,
                cartesian_point.x - closest_point_on_linestring.x,
            )
        )
    )
    relative_heading = (
        heading_of_line_joining_input_points_and_its_closest_point_on_linestring
        - local_section_heading_in_cartesian_coordinates
    )
    if relative_heading < 0 or relative_heading > 180:
        d = -1 * d

    return s, d, heading_relative_to_local_section_heading, closest_point_on_linestring


def get_cartesian_from_frenet(
    linestring: LineString,
    frenet_point: List[float],
    frenet_heading: float,
    resolution: float = 0.01,
) -> Tuple[float, float, float]:

    """
    Converts a point (s,d,frenet_heading) in the frenet frame to it's cartesian_frame representation.
    :param linestring:
        A shapely 'LineString' (Polyline) that represents the centre line of the lane being used as
        the reference for the frenet frame.
    :param frenet_point:
        A list of the form [s,d].
    :param frenet_heading:
        The heading associated with the frenet point, in degrees, in the frenet frame.
    :param resolution:
        The resolution used to calculate the heading direction of local sections of the lane LineString.
    :returns:
        Tuple of (x, y, heading), in the cartesian frame.
    """

    s = frenet_point[0]
    d = frenet_point[1]

    # Find point on the lane LineString at a runlength of s.
    point_on_linestring = linestring.interpolate(s)

    # Find heading direction of the lane LineString by using a small section of the LineString,
    # around point_on_linestring computed above.
    local_section_of_spline_start = linestring.interpolate(max(0, s - resolution))
    local_section_of_spline_end = linestring.interpolate(
        min(linestring.length, s + resolution)
    )
    local_section_heading_in_cartesian_coordinates = np.degrees(
        np.arctan2(
            local_section_of_spline_end.y - local_section_of_spline_start.y,
            local_section_of_spline_end.x - local_section_of_spline_start.x,
        )
    )

    # Get the cartesian point at offset by d, along a direction perpendicular to the heading of the local section of the lane LineString.
    angle_to_extend = (
        (local_section_heading_in_cartesian_coordinates + 90) * np.pi / 180
    )
    cartesian_point = [
        point_on_linestring.x + d * np.cos(angle_to_extend),
        point_on_linestring.y + d * np.sin(angle_to_extend),
    ]

    # Get the heading in cartesian frame.
    cartesian_heading = local_section_heading_in_cartesian_coordinates + frenet_heading

    return cartesian_point[0], cartesian_point[1], cartesian_heading
