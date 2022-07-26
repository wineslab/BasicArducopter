#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2022, Northeastern University
Created by Davide Villa (villa.d@northeastern.edu)
"""

from argparse import ArgumentParser
from time import sleep
from BasicArdu.BasicArdu import BasicArdu, Frames

# Parameters
DRONE_HEIGHT = -5       # meters
STEP = 3                # offset of each step (in meters)
NUM_STEPS = 3           # number of steps to perform

def main():
    """
    Nicholas experiment with a user-defined offset with the BasicArdu wrapper
    """

    parser = ArgumentParser()
    # parser.add_argument('--connection_string', type=str, default='tcp:127.0.0.1:5762', help='Ardupilot connection string')
    parser.add_argument('--connection_string', type=str, default='tcp:192.168.10.110:5760', help='Ardupilot connection string')
    options = parser.parse_args()

    # Simple use example
    drone = BasicArdu(connection_string=options.connection_string)    # connect to ArduPilot

    # Printing coordinates
    coords = drone.get_LLA()
    print("Coordinates home location:")
    print(coords)

    # Takeoff drone
    drone.handle_takeoff(5)
    sleep(5)

    # Going to each waypoint in a straight line towards North
    for i in range(NUM_STEPS):
        # Go to i-th waypoint - i*step m (i*step m north, 0 meters east, drone_height meters up, facing North)
        print("Going to waypoint " + str(i+1) + " ...")
        drone.handle_waypoint(Frames.NED, STEP*(i+1), 0, DRONE_HEIGHT, 0)
        print("Reached waypoint " + str(i+1) + ".")
        sleep(5)

    # Return to Home (0m north, 0 meters east, drone_height meters up, facing North)
    print("Returning home...")
    drone.handle_waypoint(Frames.NED, 0, 0, DRONE_HEIGHT, 0)
    print("Reached home.")
    sleep(5)

    # Land
    drone.handle_landing()

    print("Done!")


if __name__ == '__main__':
    main()

