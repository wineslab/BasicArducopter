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
    # parser.add_argument('--connection_string', type=str, default='tcp:127.0.0.1:5762', help='Ardupilot connection string')
    parser.add_argument('--connection_string', type=str, default='tcp:192.168.10.110:5760', help='Ardupilot connection string')
    options = parser.parse_args()

    # Simple use example
    drone = BasicArdu(connection_string=options.connection_string)    # connect to ArduPilot

    # Arming drone
    drone.handle_arm()
    sleep(10)

    # Printing coordinates
    coords = drone.get_LLA()
    print("Coordinates: ")
    print(coords)

    # Disarming drone by landing
    drone.handle_landing()
    sleep(5)


if __name__ == '__main__':
    main()

