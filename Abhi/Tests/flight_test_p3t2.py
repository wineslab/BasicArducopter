###################################################################################################
# Copyright © Abhimanyu Venkatraman Sheshashayee <abhi.vs@outlook.com>. All rights reserved.      #
# Please refer to the LICENCE file in the project root for information on use and distribution.   #
# Unauthorised use of this file is strictly prohibited.                                           #
# Unauthorised distribution of this file, via any medium, is strictly prohibited.                 #
###################################################################################################

### This module is dependent on the 'BasicArducopter' project by John Buczek.

### Imports ----------
from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames

### Global variables and constants ----------
flight_altitude_in_metres = 1
waypoints = list()
waypoints.append( ( None , None ) ) # ( latitude , longitude )
# Add more waypoints here...

### Main function ----------
def main():

	## Setup drone.
	print( 'Basic Drone is starting' )
	drone = BasicArdu( connection_string = '/dev/ttyACM0' ) # connect to Intel Aero using 'North, East, Down' Reference Basis

	## Record Home Lat / Lon / MSL.
	home_lat, home_lon, home_alt = drone.get_LLA()
	print( 'Home coordinates: ' )
	print( '\tLatitude: ' , home_lat )
	print( '\tLongitude: ' , home_lon )
	print( '\tAltitude: ' , home_alt )
	waypoint[ 0 ] = ( home_lat , home_lon )

	## Takeoff.
	print( 'Drone is taking off' )
	drone.handle_takeoff( flight_altitude_in_metres )

	### Travel to waypoints.
	for waypoint in waypoints[ 1: ]:
		print( 'Drone is travelling to Waypoint ' , str( waypoint ) )
		drone.handle_waypoint( Frames.LLA , waypoint[ 0 ] , waypoint[ 1 ] , home_alt + flight_altitude_in_metres )

	### Return home.
	print( 'Drone is returning home' )
	drone.handle_waypoint( Frames.LLA , home_lat , home_lon , flight_altitude_in_metres )

	### Land.
	print( 'Drone is landing' )
	drone.handle_landing()

### Program entry point ----------
if __name__ == '__main__':
	main()
