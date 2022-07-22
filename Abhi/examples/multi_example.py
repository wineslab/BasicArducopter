#!/usr/bin/env python3

from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames
from time import sleep

if __name__ == "__main__":
    # Vehicles in gazebo and real life set their home location (lat/lon) and ekf origin (dNorth/dEast coordinate system) when and where they are turned on
    # For multiple drones:
    # set them both with the same global home so that NED commands are all relative to the same position

    # example to be run using the simulator command: ./launch_gazebo.sh 2 "Red,Purple" "0.0,15.0" "10.0,0.0"
    v1 = BasicArdu( connection_string='tcp:192.168.10.138:5762', global_home=[42.47777625687639,-71.19357940183706,174.0])       
    v2 = BasicArdu( connection_string='tcp:192.168.10.138:5772', global_home=[42.47777625687639,-71.19357940183706,174.0])


    # takeoff v1
    v1.handle_takeoff(5)   
    sleep(3)
    # goto first waypoint
    v1.handle_waypoint(Frames.NED, 6, 0, -5, 0)
    sleep(3)

    # goto second wayoint
    v1.handle_waypoint(Frames.NED, 0, 5, -5, 0)
    sleep(3)

    # goto Home wayoint
    v1.handle_waypoint(Frames.NED, 0, 0, -5, 0)

    # land
    v1.handle_landing()


    # takeoff v2
    v2.handle_takeoff(5)   
    sleep(3)
    # goto first waypoint
    v2.handle_waypoint(Frames.NED, 6, 0, -5, 0)
    sleep(3)

    # goto second wayoint
    v2.handle_waypoint(Frames.NED, 0, 5, -5, 0)
    sleep(3)

    # goto Home wayoint
    v2.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    sleep(3)

    # goto Landing wayoint
    v2.handle_waypoint(Frames.NED, 0, -2, -5, 0)
    sleep(3)

    # land
    v2.handle_landing()