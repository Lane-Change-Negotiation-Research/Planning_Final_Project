# Miscellaneous utility functions

import math
import numpy as np
import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('/opt/CARLA_0.9.9.4/PythonAPI/carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def change_to_Town06():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town06')
    # world = client.get_world()
    weather = world.get_weather()
    weather.cloudiness = 0
    weather.sun_altitude_angle = 45
    weather.precipitation_deposits = 0
    world.set_weather(weather)
    print('\ndone.')


def get_speed(vehicle):
    """
    Compute speed of a vehicle in Km/h.
        :param vehicle: the vehicle for which speed is calculated
        :return: speed as a float in Km/h
    """
    vel = vehicle.get_velocity()

    return 3.6 * math.sqrt(vel.x ** 2 + vel.y ** 2 + vel.z ** 2)
