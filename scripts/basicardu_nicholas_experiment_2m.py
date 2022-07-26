#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2022, Northeastern University
Created by Davide Villa (villa.d@northeastern.edu)
"""

from argparse import ArgumentParser
from time import sleep
from BasicArdu.BasicArdu import BasicArdu, Frames


def main():
    """
    Nicholas experiment with a hard-coded offset of 2m with the BasicArdu wrapper
    """

    parser = ArgumentParser()
    # parser.add_argument('--connection_string', type=str, default='tcp:127.0.0.1:5762', help='Ardupilot connection string')
    parser.add_argument('--connection_string', type=str, default='tcp:192.168.10.110:5760', help='Ardupilot connection string')
    options = parser.parse_args()

    # Parameters
    drone_height = -5       # meters

    # Simple use example
    drone = BasicArdu(connection_string=options.connection_string)    # connect to ArduPilot

    # Printing coordinates
    coords = drone.get_LLA()
    print("Coordinates home location:")
    print(coords)

    # Takeoff drone
    drone.handle_takeoff(5)
    sleep(5)

    # Go to 1st waypoint - 2m (2m north, 0 meters east, 5 meters up, facing North)
    print("Going to 1st waypoint...")
    drone.handle_waypoint(Frames.NED, 10, 0, drone_height, 0)
    print("Reached 1st waypoint.")
    sleep(5)

    """
    # Go to 2nd waypoint - 4m (4m north, 0 meters east, 5 meters up, facing North)
    print("Going to 2nd waypoint...")
    drone.handle_waypoint(Frames.NED, 4, 0, drone_height, 0)
    print("Reached 2nd waypoint.")
    sleep(10)

    # Go to 3rd waypoint - 6m (6m north, 0 meters east, 5 meters up, facing North)
    print("Going to 3rd waypoint...")
    drone.handle_waypoint(Frames.NED, 6, 0, drone_height, 0)
    print("Reached 3rd waypoint.")
    sleep(10)

    # Go to 4th waypoint - 8m (8m north, 0 meters east, 5 meters up, facing North)
    print("Going to 4th waypoint...")
    drone.handle_waypoint(Frames.NED, 8, 0, drone_height, 0)
    print("Reached 4th waypoint.")
    sleep(10)

    # Go to 1st waypoint - 10m (10m north, 0 meters east, 5 meters up, facing North)
    print("Going to 5th waypoint...")
    drone.handle_waypoint(Frames.NED, 10, 0, drone_height, 0)
    print("Reached 5th waypoint.")
    sleep(10)
    """

    # Return to Home (0m north, 0 meters east, 5 meters up, facing North)
    print("Returning home...")
    drone.handle_waypoint(Frames.NED, 0, 0, drone_height, 0)
    print("Reached home.")
    sleep(5)


    # Land
    drone.handle_landing()

    print("Done!")


if __name__ == '__main__':
    main()

