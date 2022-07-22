from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames

# Main Method
def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone  = BasicArdu(connection_string='tcp:127.0.0.1:5760')    # connect to Intel Aero using 'North, East, Down' Reference Basis


    # Record Home Lat / Lon / MSL
    home_lat, home_lon, landed_alt = drone.get_LLA()

    # takeoff drone
    drone.handle_takeoff(5)  # takeoff alititude: 5 meters

    
    # goto first waypoint
    drone.handle_waypoint(Frames.LLA, home_lat+0.0002, home_lon, landed_alt + 5, 0)    # Latitude 1, Longitude 1, 5 meters above the landed altitude, Yaw angle 0rad (North) 
    # ... Code to run at first waypoint ...


    # goto second wayoint
    drone.handle_waypoint(Frames.LLA, home_lat-0.0002, home_lon, landed_alt + 5, 3.14/2) # Latitude 2, Longitude 2, 5 meters above the landed altitude, Yaw angle pi/2 rad (East)
    # ... Code to run at second waypoint ...

    # Repeat for as many waypoints as needed, or forever
    # . . .
    # . . .
    # . . .
    
    # goto Home wayoint (starting position)
    drone.handle_waypoint(Frames.LLA, home_lat, home_lon, landed_alt + 5, 0)
    

    # land
    drone.handle_landing()
    

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly (python3 filename.py)