#!/usr/bin/env python3

'''
Method to takeoff drone and ppilot it to the home location
'''

from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames
from time import sleep

if __name__ == "__main__":
    v1 = BasicArdu(frame=Frames.LLA, connection_string='tcp:192.168.10.138:5762')       

    while True:
        print(v1.get_LLA())
        sleep(2)

