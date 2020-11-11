import glob
import os
import sys
import time

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla


def main():
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.load_world('Town06')
    # world = client.get_world()
    weather = world.get_weather()
    weather.cloudiness = 0
    weather.sun_altitude_angle = 45
    weather.precipitation_deposits = 0
    world.set_weather(weather)

    # Spawnpoint manipulation
    # spawn_points = world.get_map().get_spawn_points()
    # for spawn_point in spawn_points:
    #     print(spawn_point, '\n')


if __name__ == '__main__':

    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        print('\ndone.')
