#!/usr/bin/env python

# Copyright (c) 2025 Computer Vision Center (CVC) at the Universitat Autonoma de
# Barcelona (UAB).
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import argparse
import json
import logging
import math
import signal
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

    return  world.spawn_actor(
        bp,
        map_.get_spawn_points()[0],
        attach_to=None)


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


def _update_spectator(world, vehicle, distance=6.0, height=3.0, pitch=-15.0):
    """Move the spectator to a third-person chase view behind the vehicle."""
    transform = vehicle.get_transform()
    yaw = transform.rotation.yaw

    # Unit forward vector of the vehicle (CARLA's left-handed UE convention).
    forward = carla.Vector3D(
        x=math.cos(math.radians(yaw)),
        y=math.sin(math.radians(yaw)),
        z=0.0
    )

    location = transform.location - forward * distance
    location.z += height

    spectator_transform = carla.Transform(
        location=location,
        rotation=carla.Rotation(pitch=pitch, yaw=yaw, roll=0.0)
    )

    world.get_spectator().set_transform(spectator_transform)


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
        settings.fixed_delta_seconds = 1.0 / 20.0  # 20 Hz base tick
        world.apply_settings(settings)

        applied = world.get_settings()
        logging.info(
            "Applied settings -- synchronous_mode=%s fixed_delta_seconds=%s",
            applied.synchronous_mode, applied.fixed_delta_seconds
        )

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        vehicle = _setup_vehicle(world, config)
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        _ = world.tick()

        vehicle.set_autopilot(config.get("autopilot", False))

        logging.info("Running...")

        target_dt = settings.fixed_delta_seconds
        next_tick_at = time.perf_counter()

        tick_count = 0
        window_start = time.perf_counter()
        LOG_EVERY_N_TICKS = 100

        while True:
            _ = world.tick()
            _update_spectator(world, vehicle)

            tick_count += 1
            if tick_count % LOG_EVERY_N_TICKS == 0:
                now = time.perf_counter()
                measured_hz = LOG_EVERY_N_TICKS / (now - window_start)
                logging.info(
                    "Measured client tick rate: %.2f Hz (target %.2f Hz)",
                    measured_hz, 1.0 / target_dt
                )
                window_start = now

            # Sync mode advances sim time by target_dt per tick but does not
            # pace itself to real time -- without this, a light scene ticks
            # faster than real-time and every sensor's Hz scales up with it.
            next_tick_at += target_dt
            sleep_time = next_tick_at - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                # We're behind real time (server can't keep up at this rate);
                # reset the reference instead of trying to "catch up" in a burst.
                next_tick_at = time.perf_counter()

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

    # Containers stop with SIGTERM; translate it into KeyboardInterrupt so the
    # cleanup in main() runs (destroy actors, restore world settings).
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)

    main(args)
