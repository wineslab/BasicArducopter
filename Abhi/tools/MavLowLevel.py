### import libraries
from dronekit import VehicleMode, connect, LocationGlobal, LocationLocal
from pymavlink import mavutil

def waypoint_cmd_LLA(vehicle, lat, lon, alt, yaw=0):
	'''
	Function for adding next waypoint using global conditions
	:param vehicle: dronekit vehicle object
	:param lat: float of latitude
	:param lon: float of longitude
	:param alt: float of altitude msl in meters
	:param yaw: float of the desired yaw angle (radians) 0-2pi (0 is north , pi/2 is east)
	'''
	frame = mavutil.mavlink.MAV_FRAME_GLOBAL_INT	# Global frame (Lat, Lon, Alt_msl)
	type_mask=  3064
	msg = vehicle.message_factory.set_position_target_global_int_encode(
        0, 0, 0,
        frame,
        type_mask,  # typemask what should be ignored (3576 uses position data only)
        int(lat * 1e7),  # latitude coord NEED TO BE INT
        int(lon * 1e7),  # long coord
        alt,  # altitude

        0,  # x velocity
        0,  # y velocity
        0,  # z velocity

        0,  # x accel
        0,  # y accel
        0,  # z accel

        yaw,  # yaw
        0)  # yaw rate
	# print('cmd ' + str(msg))
	vehicle.send_mavlink(msg)


def waypoint_cmd_NED(vehicle,  meters_north, meters_east, meters_down, yaw=0):
	'''
	Function for adding next waypoint using relative coordinal distances (0, 0, 0 is the home location)
	:param vehicle: dronekit vehicle object
	:param meters_north: float of meters north of the home location
	:param meters_east: float of meters east of the home location
	:param meters_down: float of meters down from the home location
	:param yaw: float of the desired yaw angle (radians) 0-2pi (0 is north , pi/2 is east)
	'''
	type_mask = 3064
	
	frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, 0, 0,
		frame,
		type_mask,  # typemask what should be ignored/used  

		meters_north,     # North position in meters
		meters_east,      # East position in meters
		meters_down,   # Alt poisiton in meters (down is positive)

		0,  # North velocity (m/s)
		0,  # East velocity (m/s)
		0,  # Alt velocity (positive is down) (m/s)

		0,  # x accel
		0,  # y accel
		0,  # z accel

		yaw,  # yaw (0 is North)
		0)  # yaw rate in radians per sec
	# print('cmd ' + str(msg))
	vehicle.send_mavlink(msg)


def kill_vehicle(vehicle, terminate=1):
	'''
	Method to terminate the flight of the vehicle immediately. Motors immediately disarm
	:param vehicle: dronekit vehicle object
	:param terminate: integer for the termination value (1: terminate, 0: lift kill switch)
	'''
	msg = vehicle.message_factory.command_long_encode(
		0, 0, mavutil.mavlink.MAV_CMD_DO_FLIGHTTERMINATION, 0,
        terminate,  # param 1: Flight termination activated if >0.5. min = 0, max = 1
        0,  # param 2: Empty
        0,  # param 3: Empty
        0,  # param 4: Empty
        0,  # param 5: Empty 
        0,  # param 6: Empty
        0   # param 7: Empty
    )
	# print('cmd ' + str(msg))
	vehicle.send_mavlink(msg)


def land_vehicle(vehicle, yaw=0, lat=0, lon=0, alt=0):
	'''
	Method to land the vehicle at its current location or at the lat/lon coordinates (if nonzero)
	:param vehicle: dronekit vehicle object
	:param lat: float of latitude
	:param lon: float of longitude
	:param alt: float of altitude msl in meters
	'''
	msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0,  # param 1: Empty
        0,  # param 2: Empty
        0,  # param 3: Empty
        yaw,    # param 4: Desired yaw angle (default 0, North)
        int(lat * 1e7),    # param 5: Target Latitude, if zero will land at current lat 
        int(lon * 1e7),      # param 6: Longitude
        alt       # param 7: Alt
    )
	# print('cmd ' + str(msg))
	vehicle.send_mavlink(msg)


def velocity_cmd_NED(vehicle, vNorth, vEast, vDown, angle=0):
	'''
	Method to send velocity movement commands 
	:param vehicle: dronekit vehicle object
	:param vNorth: float for the velocity north to reach
	:param vEast: float of the velocity east to reach
	:param vDown: float of the velocity down to reach
	:param yaw: float of the desired yaw angle (radians) 0-2pi (0 is north , pi/2 is east)
	'''
	msg = vehicle.message_factory.set_position_target_local_ned_encode(					
		0,       # time_boot_ms (not used)
		0, 0,    # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
		0b0000101111000111, # type_mask (only speed enabled)
		0, 0, 0, # x, y, z positions (or lat, lon, alt in the MAV_FRAME_BODY_NED frame (not used)
		vNorth, vEast, vDown, # x, y, z velocity in m/s  
		0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		angle, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
	
	# send command to vehicle
	# print('cmd ' + str(msg))
	vehicle.send_mavlink(msg)


def land_vehicle(vehicle, yaw=0, lat=0, lon=0, alt=0):
	'''
	Function will land the vehicle at its current location or at the lat/lon coordinates provided (if non-zero) 
	This is equivalent to setting the flght mode to LAND
	:param vehicle: Dronekit Vehicle class object
	:param yaw: float for yaw angle (radians)
	:param lat: float for the latitude (default 0 lands at current location)
	:param lon: float fot the longitude (default 0 lands at current location)
	:param alt: float for altitude in meters msl (default 0 lands at current location)
	'''
	msg = vehicle.message_factory.command_long_encode(
        0, 0, mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0,  # param 1: Empty
        0,  # param 2: Empty
        0,  # param 3: Empty
        yaw,    # param 4: Desired yaw angle (default 0, North)
        int(lat * 1e7),    # param 5: Target Latitude, if zero will land at current lat 
        int(lon * 1e7),      # param 6: Longitude
        alt       # param 7: Alt
    )
    #print('cmd ' + str(msg))
	vehicle.send_mavlink(msg)

def set_home(vehicle, lat, lon, alt):
	'''
	Method to set the vehicle home location to the specified waypoint
	:param vehicle: Dronekit Vehicle class object
	:param lat: float for latitude
	:param lon: float for longitude
	:param alt: float for altitude in meters msl
	'''
	vehicle.home_location = LocationGlobal(lat,lon, alt)
