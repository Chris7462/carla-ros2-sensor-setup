#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
import json
import logging
import time

import carla


def _setup_town(client, config):
    world = client.get_world()
    town = config.get("town")

    if town and not world.get_map().name.endswith(town):
        logging.info("Loading town: {}".format(town))
        world = client.load_world(town)
        world.tick()
        logging.info("Town loaded: {}".format(world.get_map().name))
    else:
        logging.info("Using current map: {}".format(world.get_map().name))

    return world


def _setup_vehicle(world, config):
    logging.debug("Spawning vehicle: {}".format(config.get("type")))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = bp_library.filter(config.get("type"))[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    # Use fixed spawn point from config if provided, otherwise use first available
    sp = config.get("spawn_point")
    if sp:
        transform = carla.Transform(
            carla.Location(x=sp["x"], y=sp["y"], z=sp["z"]),
            carla.Rotation(roll=sp.get("roll", 0.0),
                           pitch=sp.get("pitch", 0.0),
                           yaw=sp.get("yaw", 0.0))
        )
    else:
        transform = map_.get_spawn_points()[0]

    return world.spawn_actor(bp, transform, attach_to=None)


def _setup_sensors(world, vehicle, sensors_config):
    bp_library = world.get_blueprint_library()

    sensors = []
    for sensor in sensors_config:
        logging.debug("Spawning sensor: {}".format(sensor))

        bp = bp_library.filter(sensor.get("type"))[0]
        bp.set_attribute("ros_name", sensor.get("id"))
        bp.set_attribute("role_name", sensor.get("id"))
        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        wp = carla.Transform(
            location=carla.Location(
                x=sensor["spawn_point"]["x"],
                y=-sensor["spawn_point"]["y"],
                z=sensor["spawn_point"]["z"]
            ),
            rotation=carla.Rotation(
                roll=sensor["spawn_point"]["roll"],
                pitch=-sensor["spawn_point"]["pitch"],
                yaw=-sensor["spawn_point"]["yaw"]
            )
        )

        sensors.append(
            world.spawn_actor(
                bp,
                wp,
                attach_to=vehicle
            )
        )

        sensors[-1].enable_for_ros()

    return sensors


def main(args):

    world = None
    vehicle = None
    sensors = []
    original_settings = None

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(10.0)

        with open(args.file) as f:
            config = json.load(f)

        world = _setup_town(client, config)

        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.04
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        _ = world.tick()

        vehicle.set_autopilot(config.get("autopilot", False))

        # ── Spectator follow cam ──────────────────────────────────────────────
        spectator = world.get_spectator()

        logging.info("Running...")

        while True:
            t0 = time.time()

            _ = world.tick()

            # Update spectator to follow vehicle from behind
            transform = vehicle.get_transform()
            fwd = transform.get_forward_vector()
            spectator.set_transform(carla.Transform(
                transform.location + carla.Location(
                    x=fwd.x * -6,
                    y=fwd.y * -6,
                    z=3
                ),
                carla.Rotation(pitch=-10, yaw=transform.rotation.yaw)
            ))

            # Pace the loop to match fixed_delta_seconds in wall-clock time
            # so that sensor_tick values correspond to real-time frequencies
            elapsed = time.time() - t0
            sleep_time = settings.fixed_delta_seconds - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print('\nCancelled by user. Bye!')

    finally:
        if original_settings:
            world.apply_settings(original_settings)

        for sensor in sensors:
            sensor.destroy()

        if vehicle:
            vehicle.destroy()


if __name__ == '__main__':
    argparser = argparse.ArgumentParser(description='CARLA ROS2 native')
    argparser.add_argument('--host', metavar='H', default='localhost', help='IP of the host CARLA Simulator (default: localhost)')
    argparser.add_argument('--port', metavar='P', default=2000, type=int, help='TCP port of CARLA Simulator (default: 2000)')
    argparser.add_argument('-f', '--file', default='config.json', help='Configuration JSON file (default: config.json)')
    argparser.add_argument('-v', '--verbose', action='store_true', dest='debug', help='print debug information')

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    logging.info('Listening to server %s:%s', args.host, args.port)

    main(args)
