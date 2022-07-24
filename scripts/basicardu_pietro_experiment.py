#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2022, Northeastern University
Created by Davide Villa (villa.d@northeastern.edu)
"""

from argparse import ArgumentParser
from time import sleep
from BasicArdu.BasicArdu import BasicArdu


def main():
    """
    Simple arming-disarming test with the BasicArdu wrapper
    """

    parser = ArgumentParser()
    parser.add_argument('--connection_string', type=str, default='tcp:127.0.0.1:5762', help='Ardupilot connection string')
    options = parser.parse_args()

    # Simple use example
    drone = BasicArdu(connection_string=options.connection_string)    # connect to ArduPilot

    # Printing coordinates
    coords = drone.get_LLA()
    print("Coordinates home location:")
    print(coords)

    # Takeoff drone
    drone.handle_takeoff(5)
    sleep(10)

    # Disarming drone by landing
    drone.handle_landing()

    print("Done!")


if __name__ == '__main__':
    main()

