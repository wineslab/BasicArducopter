#!/usr/bin/env python3

'''
Basic Dronekit Wrapper for ArduPilot controller
'''
from argparse import ArgumentParser
from time import sleep, time
from math import pi

# Necessary For Package imports
import sys
import os
PACKAGE_PARENT = '..'
SCRIPT_DIR = os.path.dirname(os.path.realpath(os.path.join(os.getcwd(), os.path.expanduser(__file__))))
sys.path.append(os.path.normpath(os.path.join(SCRIPT_DIR, PACKAGE_PARENT)))

#from BasicArducopter.tools.MavLowLevel import *
#from BasicArducopter.tools.CommonStructs import Frames, Waypoint, xyz_to_latlon
from tools.MavLowLevel import *
from tools.CommonStructs import Frames, Waypoint, xyz_to_latlon
# from MavLowLevel import *
# from CommonStructs import Frames, Waypoint, xyz_to_latlon

class BasicArdu():
    def __init__(self, frame=Frames.LLA, verbose=False, connection_string='tcp:127.0.0.1:5762', tolerance_location=2.0, global_home=None, max_movement_dist=50):
        '''
        Dronekit wrapper class for Ardupilot
        :param frame: vehicle coordinate frame
        :param verbose: boolean for extra text outputs
        :param connection_string: string of ip address and port to connect 
        :param tolerance_location: float for tolerance for reaching waypoint (m)
        :param global_home: array of floats for the global origin of the drones [lat, lon, als (msl)]
        :param max_movement_dist: float for the maximum distance between waypoints in meters
        '''
        # Vehicle Connection 
        self.vehicle = connect(connection_string, wait_ready=False)
        
        if self.vehicle.armed:                # If the vehicle is already in the air, set to standby mode
            self.vehicle.mode = "GUIDED"

        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            sleep(2)
        
        while self.vehicle.gps_0.fix_type < 2:
            print("Waiting for GPS...:", self.vehicle.gps_0.fix_type)
            sleep(2)

        # Get Vehicle Home
        vehicle_home = self.vehicle.home_location
        if verbose:
            print("Getting Vehicle Home")
        while vehicle_home == None:
            # Download the vehicle waypoints (commands). Wait until download is complete. Necessary for self.vehicle.home_location to be set
            cmds = self.vehicle.commands # https://dronekit.netlify.com/automodule.html#dronekit.Vehicle.home_location
            cmds.download()
            cmds.wait_ready()
            vehicle_home = self.vehicle.home_location
        if verbose:
            print("Vehicle Home:", vehicle_home)

        # Set Global home location   
        if global_home == None:
            self.global_home_waypoint = Waypoint()
            self.global_home_waypoint.update(self.vehicle)
            self.global_home_waypoint.alt = vehicle_home.alt
        else:
            self.global_home_waypoint = Waypoint(x=global_home[0], y=global_home[1], z=global_home[2])


        # Initialize flight variables
        self.tolerance_location = tolerance_location # minimum distance variance to waypoint (meters)
        self.target_waypoint = Waypoint()                   # current target waypoint
        self.target_waypoint.update(self.vehicle)
        self.max_movement_dist = max_movement_dist

        self.verbose = verbose
        
        print(' - - - Initialization Successful - - -')

    def handle_arm(self):
        '''
        Method to arm the vehicle
        '''
        if self.verbose:
            print('> Arming')
        self.handle_guided()
        self.vehicle.armed=True
        
    def handle_guided(self):
        '''
        Method to set the flight mode to Offboard
        '''
        if self.verbose:
            print('> Set Offboard')

        self.vehicle.mode = "GUIDED"

    def handle_kill(self):
        '''
        Method to emergency stop motors
        '''
        kill_vehicle(self.vehicle)
        if self.verbose:
            print('> Emergency Stop')
    
    def handle_takeoff(self, alt, phi=0):
        '''
        Method to takeoff the vehicle
        '''
        print('~~ Take Off ~~')

        self.vehicle.mode = "GUIDED"
        self.vehicle.armed=True

        # We want for the motors to arm before we takeoff
        while not self.vehicle.armed:
            print("Waiting for arming...")
            sleep(0.5)

        self.vehicle.wait_simple_takeoff(alt)
         
    def handle_landing(self):
        '''
        Method to land the vehicle
        '''
        print("~~ Landing ~~")
        land_vehicle(self.vehicle)
        while self.vehicle.armed:
            sleep(0.5)

    def handle_waypoint(self, frame, x, y, z, phi=0):
        '''
        Method to send the vehicle to a new waypoint
        :param frame: Frame Enum frame of reference for the waypoint command
        x, y, z depend on the frames (NED: x:meters north, y:meters east, z: meters down) (LLA: x:Lat, y: Lon, z:alt msl meters)
        '''
        if self.verbose:
            print('> Waypoint CMD')

        if frame.value == Frames.VEL.value:   # velocity command
            print('VELOCITY')
            velocity_cmd_NED(self.vehicle, x, y, z, phi)
            self.target_waypoint=None 
        else:
            if frame.value == Frames.LLA.value: # If LLA, set target directly
                print('LLA', x, y, z, phi)
                self.target_waypoint = Waypoint( x=x, y=y, z=z, compass_angle=phi)

            elif frame.value == Frames.NED.value:   # if NED, convert to LLA
                print('NED', x, y, z, phi)
                self.target_waypoint = xyz_to_latlon(self.global_home_waypoint, x, y, z)

            if self.target_waypoint.current_distance(self.vehicle) <= self.max_movement_dist:
                waypoint_cmd_LLA(self.vehicle, self.target_waypoint.lat, self.target_waypoint.lon, self.target_waypoint.alt, phi)
            else:
                print('Cancelling Movement - Waypoint Too Far Away')
                self.target_waypoint = None

        if self.target_waypoint:
            self.wait_for_target()

    def handle_hold(self):
        '''
        Method to stop the vehicle from continuing to its current target location
        '''
        if self.verbose:
            print('> Hold')

        self.target_waypoint = Waypoint()     # Clear the target waypoint
        self.target_waypoint.update(self.vehicle)       # Update the target location to the current location of the vehicle
        # Set the vehicle to go to the target waypoint
        if self.target_waypoint.frame == Frames.LLA:
            waypoint_cmd_LLA(self.vehicle, self.target_waypoint.lat, self.target_waypoint.lon, self.target_waypoint.alt, self.target_waypoint.phi)
        
        elif self.target_waypoint.frame == Frames.NED:
            waypoint_cmd_NED(self.vehicle, self.target_waypoint.dNorth, self.target_waypoint.dEast, self.target_waypoint.dDown, self.target_waypoint.phi)

    def reached_target(self):
        '''
        Method to check if the vehicle has reached its target location
        :return: Boolean for if the target has been reached
        '''
        if self.target_waypoint:
            
            if self.target_waypoint.current_distance(self.vehicle) <= self.tolerance_location:
                self.target_waypoint = None 
            else:
                return False
        
        else:               # if the drone has reached its target, then the target waypoint is set to None
            return True

    def wait_for_target(self):
        ''' 
        Method to delay code progression until the target location has been reached
        '''
        start_time = time()
        while not self.reached_target():
            sleep(0.5)

            if time()-start_time>15.0:      # timeout for if stuck travelling to a waypoint
                print("WARNING: WAYPOINT NEVER REACHED")
                print("Distance to target:", self.target_waypoint.current_distance(self.vehicle))
                break

        if self.verbose:
            print('Reached Target')

    
    def get_LLA(self):
        '''
        Method to return the current Lattitude, Longitude, and msl Altitude of the vehicle
        :return (float) latitude, (float) longitude, (float) altitude msl
        '''
        return self.vehicle.location.global_frame.lat, self.vehicle.location.global_frame.lon, self.vehicle.location.global_frame.alt

def main():
    parser = ArgumentParser()
    parser.add_argument('--connection_string', type=str, default='tcp:192.168.10.138:5762', help='Ardupilot connection string')
    options = parser.parse_args()

    # simple use example
    drone = BasicArdu(connection_string=options.connection_string)    # connect to ArduPilot

    # takeoff drone
    drone.handle_takeoff(5)   
    sleep(3)

    # goto first waypoint (6m north, 0 meters east, 5 meters up, facing North)
    drone.handle_waypoint(Frames.NED, 6, 0, -5, 0)
    sleep(3)

    # goto second wayoint(0m north, 5 meters east, 5 meters up, facing South)
    drone.handle_waypoint(Frames.NED, 0, 5, -5, 3.14)
    sleep(3)

    # goto Home wayoint (0m north, 0 meters east, 5 meters up, facing North)
    drone.handle_waypoint(Frames.NED, 0, 0, -5, 0)
    sleep(3)

    # land
    drone.handle_landing()


if __name__ == '__main__':
    main()













