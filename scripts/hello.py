#!/usr/bin/env python
# -*- coding: utf-8 -*-

# print("Start simulator (SITL)")
# import dronekit_sitl
# sitl = dronekit_sitl.start_default()
# connection_string = sitl.connection_string()
conn_string = 'tcp:127.0.0.1:5762'

# Import DroneKit-Python
from dronekit import connect, VehicleMode

# Connect to the Vehicle.
print("Connecting to vehicle on: %s" % (conn_string,))
vehicle = connect(conn_string, wait_ready=True)

# Get some vehicle attributes (state)
print("Get some vehicle attribute values:")
print(" GPS: %s" % vehicle.gps_0)
print(" Battery: %s" % vehicle.battery)
print(" Last Heartbeat: %s" % vehicle.last_heartbeat)
print(" Is Armable?: %s" % vehicle.is_armable)
print(" System status: %s" % vehicle.system_status.state)
print(" Mode: %s" % vehicle.mode.name)    # settable

# Close vehicle object before exiting script
vehicle.close()

# Shut down simulator
# sitl.stop()
print("Completed")
