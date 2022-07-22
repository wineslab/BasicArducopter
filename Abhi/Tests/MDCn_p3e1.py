###################################################################################################
# Copyright © Abhimanyu Venkatraman Sheshashayee <abhi.vs@outlook.com>. All rights reserved.      #
# Please refer to the LICENCE file in the project root for information on use and distribution.   #
# Unauthorised use of this file is strictly prohibited.                                           #
# Unauthorised distribution of this file, via any medium, is strictly prohibited.                 #
###################################################################################################

### This module is dependent on:
### * 'MDCn_Code_Backup' by Abhimanyu Venkatraman Sheshashayee.
### * 'BasicArducopter' by John Buczek.


### Debug flags ----------
debug_comm = False
debug_flight = False
verbose = False


### Imports ----------

## Unconditional imports.
from collections import OrderedDict
import csv
import datetime
from enum import Enum , unique #, auto
import json
import math
import os
import struct
import sys
import threading
from time import sleep


## Conditional imports.
if not debug_comm:
	from tinyos3 import tos
if not debug_flight:
	from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames

### Global variables and constants ----------

## Control constants (may be altered before program execution, but must not change during runtime).
serial_port_numbers = [ i for i in range( 10 ) ]		# Port numbers to check for serial connection to MagoNode++.
SN_IDs = [ 2 , 3 , 4 , 5 , 6 , 7 , 8 , 9 , 10 , 11 ]	# Shortened IDs for deployed sensor nodes.
max_number_of_attempts = 20						# The maximum number of times that a node will try to send a message.
milliseconds_between_attempts = 50					# The number of milliseconds between consecutive attempts to send a message. Must be greater than or equal to 25.
scenario_WuR = True								# Experiment scenario flag: if true, then the SNs respond to WuSs; if false, then the SNs duty cycle and respond to CTSs.
DC_cycle_period_in_milliseconds = 1000				# Duty cycle net duration (awake and asleep). Must be greater than 25.
DC_awake_period_in_milliseconds = 100				# Duty cycle awake duration. Must be greater than or equal to 0, and less than or equal to DC_cycle_period_in_milliseconds.
flight_altitude_in_metres = 5						# Expected altitude of the drone, in metres. Must be greater than or equal to 1.
data_filename = 'MDCn_naive.json'					# Filename of file to which data is to be saved.

## Non-control constants (must not be altered, must not change during runtime).
@unique
class Nodes( Enum ):
	MDC_SNIFFER = 0	# Mobile Data Collector Sniffer mote.
	MDC_WUR = 1		# Mobile Data Collector Wake-up Radio mote.
	SN_WUR = 2		# Sensor Node with attached Wake-up Radio. Sensor nodes can have shortened IDs that are >=2.
@unique
class PacketTypes( Enum ):
	SET_SCENARIO = 5	# MDC_Sniffer tells SN_WuR to set its scenario variables.
	ACK_SCENARIO = 6	# SN_WuR responds to SET_SCENARIO with an acknowledgement that also carries relevant metrics.
	CTS = 7		# MDC_Sniffer tells SN_WuR to send a data packet to MDC_Sniffer.
	WUS = 8		# MDC_Sniffer tells MDC_WuR to broadcast Wake-up Sequence, which in turn tells SN_WuR to send a data packet to MDC_Sniffer.
	DATA = 9			# SN_WuR sends data packet to MDC_Sniffer.
class ScenarioTypes( Enum ):
	SCENARIO_OFF = 15	# SN_WuR does not duty cycle and does not respond to either WuS or CTS. ACK_SCENARIO returns milliseconds awake.
	SCENARIO_WUR = 16	# SN_WuR does not duty cycle and responds to WuS.
	SCENARIO_DC = 17	# SN_WuR duty cycles and responds to CTS. SET_SCENARIO sets duty cycle.
RADIO_AM_CONTROL_T = 23	# Primary radio active message control number.
SERIAL_AM_CONTROL_T = 88	# Serial active message control number.
if not debug_comm:
	class Serial_AM_Packet( tos.Packet ):
		def __init__( self , packet = None ):
			variables = 	[
							( 'source' , 'int' , 1 )
							, ( 'packet_type' , 'int' , 1 )
							, ( 'WuS' , 'int' , 1 )
							, ( 'scenario_type' , 'int' , 1 )
							, ( 'cycle_period' , 'int' , 2 )
							, ( 'awake_period' , 'int' , 2 )
							, ( 'ms_awake' , 'int' , 4 )
						]
			tos.Packet.__init__( self , variables , packet )
assert 50 <= DC_cycle_period_in_milliseconds
assert 0 <= DC_awake_period_in_milliseconds <= DC_cycle_period_in_milliseconds
assert 25 < milliseconds_between_attempts
assert 1 <= flight_altitude_in_metres

## Variables (must not be altered, may change during runtime).
collection_cycle_start_time = None
collection_cycle_end_time = None
latencies = [ list() for _ in range( len( SN_IDs ) ) ]
awake_durations = [ None for _ in range( len( SN_IDs ) ) ]
go_to_next_waypoint = True


### Utility functions ----------
def get_EST():
	return datetime.datetime.now( datetime.timezone( datetime.timedelta( hours = -4 ) ) )

def get_coordinates():
	coords = [ 0 , 0 , 0 ]
	if not debug_flight:
		try:
			drone = BasicArdu( frame = Frames.LLA , connection_string = '/dev/ttyACM0' )
			lat , lon , alt = drone.get_LLA()
			coords = [ lat , lon , alt ]
		except Exception as e:
			pass
	return coords

def save_orienting_coordinates( coords , filename = 'orienting_coords.csv' ):
	if os.path.exists( filename ):
		os.remove( filename )
	with open( filename , 'w' ) as file:
		file.write( str( coords[ 0 ] ) + ',' + str( coords[ 1 ] ) )

def load_orienting_coordinates( filename = 'orienting_coords.csv' ):
	coords = [ 0 , 1 ]
	if os.path.exists( filename ):
		with open( filename , 'r' ) as file:
			data = list( csv.reader( file , quoting = csv.QUOTE_NONNUMERIC ) )
			coords[ 0 ] = data[ -1 ][ 0 ]
			coords[ 1 ] = data[ -1 ][ 1 ]
	return coords


### Waypoint functions ----------

## Generates waypoints for a clockwise square spiral path.
def generate_square_spiral_path( layers = 2 , origin = [ 0 , 0 ] ):

	if layers <= 0:
		return None

	total_number_of_path_waypoints = ( ( layers * 2 ) + 1 ) ** 2
	x = [ None for _ in range( total_number_of_path_waypoints ) ]
	y = [ None for _ in range( total_number_of_path_waypoints ) ]

	x[ 0 ] = origin[ 0 ]
	y[ 0 ] = origin[ 1 ]

	for n in range( total_number_of_path_waypoints ):
		crn = math.ceil( math.sqrt( n + 1 ) )
		layer_diameter = ( crn + 1 ) if ( ( crn % 2 ) == 0 ) else crn
		layer_radius = ( layer_diameter - 1 ) / 2
		layer_n = n + 1 - ( ( layer_diameter - 2 ) ** 2 )
		if layer_n == 0:
			pass
		elif layer_n <= layer_diameter - 1:
			x[ n ] = x[ 0 ] - layer_radius + layer_n
			y[ n ] = y[ 0 ] + layer_radius
		elif layer_n <= ( 2 * layer_diameter ) - 2:
			x[ n ] = x[ 0 ] + layer_radius
			y[ n ] = y[ 0 ] + layer_radius - layer_n + layer_diameter - 1
		elif layer_n <= ( 3 * layer_diameter ) - 3:
			x[ n ] = x[ 0 ] + layer_radius - ( layer_n - ( 2 * layer_diameter ) + 2 )
			y[ n ] = y[ 0 ] - layer_radius
		elif layer_n <= ( 4 * layer_diameter ) - 4:
			x[ n ] = x[ 0 ] - layer_radius
			y[ n ] = y[ 0 ] - layer_radius + layer_n - ( 3 * layer_diameter ) + 3

	path_waypoints = [ [ None , None ] for _ in range( total_number_of_path_waypoints ) ]
	for n in range( total_number_of_path_waypoints ):
		path_waypoints[ n ] = ( x[ n ] , y[ n ] )

	return path_waypoints

def scale_waypoints_by_factor( path_waypoints , scaling_factor = 1 , origin = [ 0 , 0 ] ):
	new_path_waypoints = [ [ None , None ] for _ in range( len( path_waypoints ) ) ]
	for i in range( len( path_waypoints ) ):
		new_path_waypoints[ i ][ 0 ] = ( scaling_factor * ( path_waypoints[ i ][ 0 ] - origin[ 0 ] ) ) + origin[ 0 ]
		new_path_waypoints[ i ][ 1 ] = ( scaling_factor * ( path_waypoints[ i ][ 1 ] - origin[ 1 ] ) ) + origin[ 1 ]
	return new_path_waypoints

def scale_waypoints_to_point( path_waypoints , orienting_point = [ 0 , 1 ] , origin = [ 0 , 0 ] ):
	new_path_waypoints = [ [ None , None ] for _ in range( len( path_waypoints ) ) ]
	scaling_factor = math.sqrt( ( ( orienting_point[ 0 ] - origin[ 0 ] ) ** 2 ) + ( ( orienting_point[ 1 ] - origin[ 1 ] ) ** 2 ) )
	return scale_waypoints_by_factor( path_waypoints , scaling_factor , origin )

def rotate_waypoints_by_angle( path_waypoints , angle = 0 , origin = [ 0 , 0 ] ):
	new_path_waypoints = [ [ None , None ] for _ in range( len( path_waypoints ) ) ]
	for i in range( len( path_waypoints ) ):
		new_path_waypoints[ i ][ 0 ] = origin[ 0 ] + ( math.cos( -angle ) * ( path_waypoints[ i ][ 0 ] - origin[ 0 ] ) ) - ( math.sin( -angle ) * ( path_waypoints[ i ][ 1 ] - origin[ 1 ] ) )
		new_path_waypoints[ i ][ 1 ] = origin[ 1 ] + ( math.sin( -angle ) * ( path_waypoints[ i ][ 0 ] - origin[ 0 ] ) ) + ( math.cos( -angle ) * ( path_waypoints[ i ][ 1 ] - origin[ 1 ] ) )
	return new_path_waypoints

def rotate_waypoints_to_point( path_waypoints , orienting_point = [ 0 , 1 ] , origin = [ 0 , 0 ] ):
	new_path_waypoints = [ [ None , None ] for _ in range( len( path_waypoints ) ) ]
	angle = math.atan2( orienting_point[ 0 ] - origin[ 0 ] , orienting_point[ 1 ] - origin[ 1 ] )
	return rotate_waypoints_by_angle( path_waypoints , angle , origin )


### Communication functions ----------

## Initialise the serial port.
def initialise_serial_port():
	serial_port = None
	sys.stdout = open( os.devnull , 'w' )
	for i in serial_port_numbers:
		try:
			serial_port = tos.getSource( 'serial@/dev/ttyUSB' + str( i ) + ':57600' )
			break
		except:
			pass
	sys.stdout = sys.__stdout__
	if serial_port == None:
		print( get_EST() , '|' , 'Error: Failed to initialise the serial port.' , flush = True )
	return serial_port

## Initialise the active message controller.
def initialise_active_message_controller( serial_port ):
	active_message_controller = None
	if serial_port != None:
		try:
			active_message_controller = tos.AM( serial_port )
		except Exception as e:
			print( get_EST() , '|' , 'Error: Failed to initialise the active message controller.' , flush = True )
			if verbose:
				print( e )
	else:
		print( get_EST() , '|' , 'Error: Invalid serial port specified.' , flush = True )
	return active_message_controller

## Either:
## * Instruct the MDC_Sniffer to send a packet to the MDC_WuR that will result in a WuS being broadcast that will result in a data packet being sent back.
## * Instruct the MDC_Sniffer to send a packet to the SN_WuR to that will result in a data packet being sent back.
def send_request_for_data_packet( active_message_controller , destination_SN ):
	if active_message_controller != None:
		packet_to_send = Serial_AM_Packet()
		packet_to_send.source = Nodes.MDC_SNIFFER.value
		if scenario_WuR:
			packet_to_send.packet_type = PacketTypes.WUS.value
		else:
			packet_to_send.packet_type = PacketTypes.CTS.value
		packet_to_send.scenario_type = 0
		packet_to_send.WuS = destination_SN
		packet_to_send.cycle_period = DC_cycle_period_in_milliseconds
		packet_to_send.awake_period = DC_awake_period_in_milliseconds
		packet_to_send.ms_awake = 0
		try:
			active_message_controller.write( packet = packet_to_send , amId = SERIAL_AM_CONTROL_T )
			if verbose:
				if scenario_WuR:
					print( get_EST() , '|' , 'MDC_WuR was told to broadcast WuS' , str( destination_SN ) , flush = True )
				else:
					print( get_EST() , '|' , 'CTS sent to SN' , str( destination_SN ) , flush = True )
		except Exception as e:
			if scenario_WuR:
				print( get_EST() , '|' , 'Error: failed to send the WUS packet.' , flush = True )
			else:
				print( get_EST() , '|' , 'Error: failed to send the CTS packet.' , flush = True )
			if verbose:
				print( e , flush = True )
			print( flush = True )
	else:
		print( get_EST() , '|' , 'Error: Invalid active message controller specified.' , flush = True )

## Instruct the MDC_Sniffer to send a packet to the SN_WuR to that will result in the scenario parameters being set and an acknowledgement being sent back.
def send_scenario_packet( active_message_controller , destination_SN , scenario_type ):
	if active_message_controller != None:
		packet_to_send = Serial_AM_Packet()
		packet_to_send.source = Nodes.MDC_SNIFFER.value
		packet_to_send.packet_type = PacketTypes.SET_SCENARIO.value
		packet_to_send.scenario_type = scenario_type
		packet_to_send.WuS = destination_SN
		packet_to_send.cycle_period = DC_cycle_period_in_milliseconds
		packet_to_send.awake_period = DC_awake_period_in_milliseconds
		packet_to_send.ms_awake = 0
		try:
			active_message_controller.write( packet = packet_to_send , amId = SERIAL_AM_CONTROL_T )
			if verbose:
				print( get_EST() , '|' , 'SET_SCENARIO sent to' , str( destination_SN ) , flush = True )
		except Exception as e:
			print( get_EST() , '|' , 'Error: failed to send the SET_SCENARIO packet.' , flush = True )
			if verbose:
				print( e , flush = True )
			print( flush = True )
	else:
		print( get_EST() , '|' , 'Error: Invalid active message controller specified.' , flush = True )

## Receive a serial message and extract its packet.
def receive_serial_packet( active_message_controller ):
	received_message = None
	received_packet = None
	if active_message_controller != None:
		try:
			received_message = active_message_controller.read( timeout = 0.001 ) # Timeout set to prevent blocking.
		except:
			pass
		if received_message:
			received_packet = Serial_AM_Packet( received_message.data )
	else:
		print( '\n' + 'Error: Invalid active message controller specified.' + '\n' , flush = True )
	return received_packet

## The MDC_Sniffer either:
## * Sends a packet to the MDC_WuR, which broadcasts a WuS, which tells the SN_WuR to send back a data packet.
## * Sends a packet to the SN_WuR, which tells the SN_WuR to send back a data packet.
def collect_data_from_node( active_message_controller , SN_ID ):

	# Declare/initialise variable(s).
	success = False
	attempt_counter = 0
	loop = True

	# Start the communication loop.
	attempt_start_time = datetime.datetime.now()
	while loop and ( active_message_controller != None ):

		# Get timing variables.
		current_time = datetime.datetime.now()
		seconds_since_last_attempt = ( current_time - attempt_start_time ).total_seconds()

		# Request data packet.
		if ( attempt_counter < 1 ) or ( seconds_since_last_attempt > ( milliseconds_between_attempts / 1000.0 ) ):
			attempt_start_time = current_time
			seconds_since_last_attempt = 0
			if attempt_counter < max_number_of_attempts:
				attempt_counter += 1
				send_request_for_data_packet( active_message_controller , SN_ID )
			else:
				loop = False
				if verbose:
					print( get_EST() , '|' , 'Data collection failed for node' , SN_ID , 'after' , attempt_counter , 'attempt(s).' , flush = True )

		# Receive serial packet.
		received_packet = receive_serial_packet( active_message_controller )
		if received_packet != None:
			# Process received packet.
			#print( 'Packet received: ' + str( received_packet ) , flush = True )
			if received_packet.packet_type == PacketTypes.DATA.value and received_packet.source == SN_ID:
				print( get_EST() , '|' , 'Data packet successfully received from node' , received_packet.source , 'after' , attempt_counter , 'attempt(s).' , flush = True )
				success = True
				loop = False

	# Return.
	return success

## The MDC_Sniffer sends a packet to the SN_WuR, which sets the specified scenario parameters, then returns an acknowlegement.
def set_scenario( active_message_controller , SN_ID , scenario_type ):

	# Declare/initialise variable(s).
	output = None if scenario_type == ScenarioTypes.SCENARIO_OFF.value else False
	attempt_counter = 0
	loop = True

	# Start the communication loop.
	attempt_start_time = datetime.datetime.now()
	while loop and ( active_message_controller != None ):

		# Get timing variables.
		current_time = datetime.datetime.now()
		seconds_since_last_attempt = ( current_time - attempt_start_time ).total_seconds()

		# Request data packet.
		if ( attempt_counter < 1 ) or ( seconds_since_last_attempt > ( milliseconds_between_attempts / 1000.0 ) ):
			attempt_start_time = current_time
			seconds_since_last_attempt = 0
			if attempt_counter < max_number_of_attempts:
				attempt_counter += 1
				send_scenario_packet( active_message_controller , SN_ID , scenario_type )
			else:
				loop = False
				if verbose:
					print( get_EST() , '|' , 'Scenario setting failed for node' , SN_ID , 'after' , attempt_counter , 'attempt(s).' , flush = True )

		# Receive serial packet.
		received_packet = receive_serial_packet( active_message_controller )
		if received_packet != None:
			# Process received packet.
			if received_packet.packet_type == PacketTypes.ACK_SCENARIO.value and received_packet.source == SN_ID:
				print( get_EST() , '|' , 'Scenario acknowledgement successfully received from node' , received_packet.source , 'after' , attempt_counter , 'attempt(s).' , flush = True )
				if scenario_type == ScenarioTypes.SCENARIO_OFF.value:
					output = received_packet.ms_awake
				else:
					output = True
				loop = False

	# Return.
	return output

## Initialise active message controller for serial communications.
def initialise_comm():
	serial_port = initialise_serial_port()
	active_message_controller = initialise_active_message_controller( serial_port )
	return active_message_controller


### Main functions ----------

## Initialise resource required for collection cycle.
def initialise_collection_cycle():

	# Set-up variables.
	global debug_comm
	global debug_flight
	active_message_controller = None
	drone = None

	# Set-up communications.
	print( get_EST() , '|' , 'Serial communication initialising...' , flush = True )
	if not debug_comm:
		try:
			active_message_controller = initialise_comm()
			print( get_EST() , '|' , 'Serial communication initialised.' , flush = True )
		except Exception as e:
			print( get_EST() , '|' , 'Error:' , e , flush = True )
			print( get_EST() , '|' , 'Communication debug mode activated.' , flush = True )
			#debug_comm = True
			print( get_EST() , '|' , 'Serial communication initialised in communication debug mode.' , flush = True )
	else:
		print( get_EST() , '|' , 'Serial communication initialised in communication debug mode.' , flush = True )

	# Set-up drone.
	print( get_EST() , '|' , 'Basic drone initialising...' , flush = True )
	if not debug_flight:
		try:
			drone = BasicArdu( frame = Frames.LLA , connection_string = '/dev/ttyACM0' )
			print( get_EST() , '|' , 'Basic drone initialised.' , flush = True )
		except Exception as e:
			print( get_EST() , '|' , 'Error:' , e , flush = True )
			print( get_EST() , '|' , 'Flight debug mode activated.' , flush = True )
			#debug_flight = True
			print( get_EST() , '|' , 'Basic drone initialised in flight debug mode.' , flush = True )
	else:
		print( get_EST() , '|' , 'Basic drone initialised in flight debug mode.' , flush = True )

	# Return.
	return active_message_controller , drone

## 1) Determine flight path
## 2) Take-off
## 3) Initialise experiment (signal SNs to set scenario)
## 4) Fly and collect data
## 5) Conclude experiment (signal SNs to reset scenario)
## 6) Land
def run_collection_cycle( active_message_controller , drone ):

	# Set-up variables.
	global debug_comm
	global debug_flight
	global collection_cycle_start_time
	global collection_cycle_end_time
	global latencies
	global awake_durations
	global go_to_next_waypoint
	origin = [ 0 , 0 ] # Base station position, in latitude and longitude.
	orienting_point = [ 0 , 1 ] # Point in latitude and longitude, used to determine the orientation (namely, the 'front') of the naive path.
	data_collection_start_time = None
	data_collection_end_time = None

	# Check parameter validity.
	if ( active_message_controller or debug_comm ) and ( drone or debug_flight ):

		# Determine base station coordinates.
		print( get_EST() , '|' , 'Obtaining base station coordinates...' , flush = True )
		if not debug_flight:
			try:
				home_lat, home_lon, home_alt = drone.get_LLA()
				print( get_EST() , '|' , 'Base station coordinates obtained.' , flush = True )
				print( get_EST() , '|' , '\tLatitude: ' , home_lat , flush = True )
				print( get_EST() , '|' , '\tLongitude: ' , home_lon , flush = True )
				print( get_EST() , '|' , '\tAltitude: ' , home_alt , flush = True )
				origin = [ home_lat , home_lon ]
			except Exception as e:
				print( get_EST() , '|' , 'Error:' , e , flush = True )
				print( get_EST() , '|' , 'Flight debug mode activated.' , flush = True )
				debug_flight = True
				print( get_EST() , '|' , 'Base Station coordinates unavailable in flight debug mode.' , flush = True )
		else:
			print( get_EST() , '|' , 'Base Station coordinates unavailable in flight debug mode.' , flush = True )

		# Determine orienting coordinates.
		orienting_point = load_orienting_coordinates()

		# Determine collection points.
		print( get_EST() , '|' , 'Generating collection points...' , flush = True )
		collection_points = generate_square_spiral_path( 1 , origin )
		collection_points = rotate_waypoints_to_point( collection_points , orienting_point , origin )
		collection_points = scale_waypoints_to_point( collection_points , orienting_point , origin )
		collection_points = scale_waypoints_by_factor( collection_points , 1/3 , origin )
		print( get_EST() , '|' , 'Collection points generated.' , flush = True )

		## Logic for experiment initialisation goes here.
		collection_cycle_start_time = datetime.datetime.now()
		for SN_ID in SN_IDs:
			if scenario_WuR:
				success = set_scenario( active_message_controller , SN_ID , ScenarioTypes.SCENARIO_WUR.value )
			else:
				success = set_scenario( active_message_controller , SN_ID , ScenarioTypes.SCENARIO_DC.value )
			if not success:
				go_to_next_waypoint = False

		# Takeoff.
		print( get_EST() , '|' , 'MDC is taking off...' , flush = True )
		if not debug_flight:
			try:
				drone.handle_takeoff( flight_altitude_in_metres )
				print( get_EST() , '|' , 'MDC has taken off.' , flush = True )
			except Exception as e:
				print( get_EST() , '|' , 'Error:' , e , flush = True )
				print( get_EST() , '|' , 'Flight debug mode activated.' , flush = True )
				debug_flight = True
				sleep( 1 ) # Sleep for 1 seconds to represent the time taken for the MDC to take off.
				print( get_EST() , '|' , 'MDC has taken off in flight debug mode.' , flush = True )
		else:
			sleep( 1 ) # Sleep for 1 seconds to represent the time taken for the MDC to take off.
			print( get_EST() , '|' , 'MDC has taken off in flight debug mode.' , flush = True )

		# Collect data at collection points.
		for collection_point in collection_points:

			if go_to_next_waypoint:

				# Travel to next collection point.
				print( get_EST() , '|' , 'MDC is travelling to collection point...' , flush = True )
				print( get_EST() , '|' , '\tCoordinates:' , collection_point , flush = True )
				if not debug_flight:
					try:
						drone.handle_waypoint( Frames.LLA , collection_point[ 0 ] , collection_point[ 1 ] , home_alt + flight_altitude_in_metres )
						print( get_EST() , '|' , 'MDC has reached collection point.' , flush = True )
					except Exception as e:
						print( get_EST() , '|' , 'Error:' , e , flush = True )
						#print( get_EST() , '|' , 'Flight debug mode activated.' , flush = True )
						#debug_flight = True
						#print( get_EST() , '|' , 'MDC has reached collection point in flight debug mode.' , flush = True )
				else:
					sleep( 1 ) # Sleep for 1 second to represent the time taken for the MDC to fly to the collection point.
					print( get_EST() , '|' , 'MDC has reached collection point in flight debug mode.' , flush = True )
		
				# Collect data at collection point.
				print( get_EST() , '|' , 'MDC is collecting data...' , flush = True )
				for index_of_SN_ID in range( len( SN_IDs ) ):
					if go_to_next_waypoint:
						SN_ID = SN_IDs[ index_of_SN_ID ]
						success = False
						data_collection_start_time = datetime.datetime.now()
						if not debug_comm:
							try:
								success = collect_data_from_node( active_message_controller , SN_ID )
							except Exception as e:
								print( get_EST() , '|' , 'Error:' , e , flush = True )
						else:
							success = True
							sleep( 0.1 ) # Sleep for 0.1 seconds to represent the time taken for the MDC to collect data from the SN.
							print( get_EST() , '|' , 'MDC has collected data in communication debug mode.' , flush = True )
						data_collection_end_time = datetime.datetime.now()
						if success and data_collection_start_time and data_collection_end_time:
							latency = ( data_collection_end_time - data_collection_start_time ).total_seconds()
							print( get_EST() , '|' , '\tData collected from node' , SN_ID , 'in' , latency , 'seconds.' , flush = True )
							latencies[ index_of_SN_ID ].append( latency )
						#else:
						#	print( get_EST() , '|' , '\tData collection from node' , SN_ID , 'failed.' , flush = True )

			if go_to_next_waypoint:
				print( get_EST() , '|' , 'MDC has collected data.' , flush = True )

		# Return home.
		if go_to_next_waypoint:
			print( get_EST() , '|' , 'MDC is returning to base station...' )
			if not debug_flight:
				try:
					drone.handle_waypoint( Frames.LLA , home_lat , home_lon , home_alt + flight_altitude_in_metres )
					print( get_EST() , '|' , 'MDC has reached base station.' , flush = True )
				except Exception as e:
					print( get_EST() , '|' , 'Error:' , e , flush = True )
			else:
				sleep( 2 ) # Sleep for 2 seconds to represent the time taken for the MDC to fly to the base station.
				print( get_EST() , '|' , 'MDC has reached base station in flight debug mode.' , flush = True )

		# Land.
		if go_to_next_waypoint:
			print( get_EST() , '|' , 'MDC is landing...' )
			if not debug_flight:
				try:
					drone.handle_landing()
					collection_cycle_end_time = datetime.datetime.now()
					print( get_EST() , '|' , 'MDC has landed.' )
				except Exception as e:
					print( get_EST() , '|' , 'Error:' , e , flush = True )
			else:
				sleep( 1 ) # Sleep for 1 seconds to represent the time taken for the MDC to land.
				print( get_EST() , '|' , 'MDC has landed in flight debug mode.' , flush = True )
		else:
			print( get_EST() , '|' , 'MDC is emergency landing...' )
			if not debug_flight:
				try:
					drone.handle_landing()
					print( get_EST() , '|' , 'MDC has emergency landed.' )
				except Exception as e:
					print( get_EST() , '|' , 'Error:' , e , flush = True )
			else:
				sleep( 1 ) # Sleep for 1 seconds to represent the time taken for the MDC to land.
				print( get_EST() , '|' , 'MDC has emergency landed in flight debug mode.' , flush = True )

		## Logic for experiment finalisation goes here.
		for index_of_SN_ID in range( len( SN_IDs ) ):
			SN_ID = SN_IDs[ index_of_SN_ID ]
			awake_durations[ index_of_SN_ID ] = set_scenario( active_message_controller , SN_ID , ScenarioTypes.SCENARIO_OFF.value )
		collection_cycle_end_time = datetime.datetime.now()


## Collate all of the collected data and save it to file.
def finalise_collection_cycle( active_message_controller ):

	# Set-up variables.
	global collection_cycle_start_time
	global collection_cycle_end_time
	global latencies
	global awake_durations
	collection_cycle_duration = None
	reliability = None
	mean_latencies = [ None for _ in range( len( latencies ) ) ]

	# Calculate collection cycle duration.
	if collection_cycle_start_time and collection_cycle_end_time:
		collection_cycle_duration = ( collection_cycle_end_time - collection_cycle_start_time ).total_seconds()
		print( get_EST() , '|' , 'Collection cycle duration:' , collection_cycle_duration , 'seconds.' , flush = True )
	
	# Calculate reliability, latencies, and awake durations.
	if len( SN_IDs ) > 0:
		SNs_collected_from = 0
		for index_of_SN in range( len( latencies ) ):
			SN_latencies = latencies[ index_of_SN ]
			if len( SN_latencies ) > 0:
				SNs_collected_from += 1
				mean_latencies[ index_of_SN ] = sum( SN_latencies ) / len( SN_latencies )
				### DEBUG BELOW!
				if scenario_WuR:
					awake_durations[ index_of_SN ] = len( SN_latencies ) * ( 23 / 1000 )
				#else:
				#	import random
				#	mean_latencies[ index_of_SN ] += random.randint( 0 , ( DC_cycle_period_in_milliseconds - DC_awake_period_in_milliseconds ) ) / DC_cycle_period_in_milliseconds
				#	awake_durations[ index_of_SN ] = ( DC_awake_period_in_milliseconds / DC_cycle_period_in_milliseconds ) * collection_cycle_duration
				### DEBUG ABOVE!
		reliability = SNs_collected_from / len( SN_IDs )

	# Write data to file.
	entry = OrderedDict()
	entry[ 'scenario' ] = 'WuR' if scenario_WuR else ( 'DC ( ' + str( DC_awake_period_in_milliseconds ) + ' / ' + str( DC_cycle_period_in_milliseconds ) + ' )' )
	entry[ 'collection cycle start time' ] = str( collection_cycle_start_time )
	entry[ 'collection cycle end time' ] = str( collection_cycle_end_time )
	entry[ 'collection cycle duration' ] = collection_cycle_duration
	entry[ 'reliability' ] = reliability
	for index_of_SN in range( len( SN_IDs ) ):
		SN_ID = SN_IDs[ index_of_SN ]
		entry[ ( 'SN ' + str( SN_ID ) + ' - mean latency' ) ] = mean_latencies[ index_of_SN ]
	for index_of_SN in range( len( SN_IDs ) ):
		SN_ID = SN_IDs[ index_of_SN ]
		entry[ ( 'SN ' + str( SN_ID ) + ' - awake duration' ) ] = awake_durations[ index_of_SN ]

	with open( data_filename , 'ab+' ) as file:

		# Go to the end of the file.
		file.seek( 0 , 2 )

		# If the file is empty...
		if file.tell() == 0:
			# Write opening square bracket for list comprehension.
			file.write( '[\n'.encode() )

			# Write entry dictionary in json format.
			file.write( json.dumps( entry , indent = 5 ).encode() )

			# Write closing square bracket for list comprehension.
			file.write('\n]'.encode())
		else:
			# Go to two characters prior to the end of the file - these should be '\n]'
			file.seek( -2 , 2 )

			# Remove last two characters.
			file.truncate()

			# Write comma and newline to separate dictionaries.
			file.write( ',\n'.encode() )

			# Write entry dictionary in json format.
			file.write( json.dumps( entry , indent = 5 ).encode() )

			# Write closing square bracket for list comprehension.
			file.write( '\n]'.encode() )

## Thread for terminating the flight in case of emergency.
def controller( drone ):

	# Set-up variables.
	global debug_flight
	global go_to_next_waypoint

	# Blocking input with important notification.
	response = input( ( '!' * 20 ) + '\nCollection cycle starting!\nPress ENTER to terminate the collection cycle!\n' + ( '!' * 20 ) + '\n' )
	
	# Logically terminate all actions besides landing.
	print( get_EST() , '|' , 'Emergency landing triggered!' , flush = True )
	go_to_next_waypoint = False


### Program entry point ----------
if __name__ == '__main__':
	print( 'Select option:' , flush = True )
	print( '\t1) Test communications' , flush = True )
	print( '\t2) Get MDC coordinates' , flush = True )
	print( '\t3) Get orienting coordinates' , flush = True )
	print( '\t4) Perform collection cycle' , flush = True )
	user_input = input( 'Enter any other input to exit.\n' )
	if user_input == '1':
		if debug_comm:
			print( get_EST() , '|' , 'Data collection not possible in communication debug mode.' , flush = True )
		else:
			active_message_controller = initialise_comm()
			for SN_ID in SN_IDs:
				if scenario_WuR:
					success = set_scenario( active_message_controller , SN_ID , ScenarioTypes.SCENARIO_WUR.value )
				else:
					success = set_scenario( active_message_controller , SN_ID , ScenarioTypes.SCENARIO_DC.value )
			#sleep( 1 )
			for SN_ID in SN_IDs:
				print( get_EST() , '|' , 'Collecting data from SN ' + str( SN_ID ) + '...' , flush = True )
				success = collect_data_from_node( active_message_controller , SN_ID )
			for index_of_SN_ID in range( len( SN_IDs ) ):
				SN_ID = SN_IDs[ index_of_SN_ID ]
				print( 'SN' , SN_ID , 'was awake for' , set_scenario( active_message_controller , SN_ID , ScenarioTypes.SCENARIO_OFF.value ) , 'milliseconds.' )
	elif user_input == '2':
		print( 'MDC coordinates:' , get_coordinates() , flush = True )
	elif user_input == '3':
		save_orienting_coordinates( get_coordinates() )
	elif user_input == '4':
		active_message_controller , drone = initialise_collection_cycle()
		
		if ( active_message_controller or debug_comm ) and ( drone or debug_flight ):

			control_thread = threading.Thread( target = controller , args = ( drone , ) )
			collection_thread = threading.Thread( target = run_collection_cycle , args = ( active_message_controller , drone ) )

			control_thread.daemon = True
			control_thread.start()
			#collection_thread.daemon = True
			collection_thread.start()

			#control_thread.join()
			collection_thread.join()

			if go_to_next_waypoint:
				finalise_collection_cycle( active_message_controller )
