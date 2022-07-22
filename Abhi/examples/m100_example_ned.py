from BasicArducopter.BasicArdu.BasicArdu import BasicArdu, Frames

# Main Method
def main():
    # simple use example
    print('---Starting Basic Drone---')
    drone  = BasicArdu(connection_string="")#'/dev/ttyACM0')    # connect to M100 (USB) 


    # takeoff drone
    drone.handle_takeoff(5)  # takeoff alititude: 5 meters

    
    # goto first waypoint
    drone.handle_waypoint(Frames.NED, 10.0, 0, -5.0, 0)    # 10 meters North, 0 meters East, -5 meters Down, Yaw angle 0rad (North) 
    # ... Code to run at first waypoint ...


    # goto second wayoint
    drone.handle_waypoint(Frames.NED, 0, 5.0, -5.0, 3.14/2) # 0 meters North, 5 meters East, -5 meters Down, Yaw angle pi/2 rad (East)
    # ... Code to run at second waypoint ...

    # Repeat for as many waypoints as needed, or forever
    # . . .
    # . . .
    # . . .
    
    # goto Home wayoint (starting position)
    drone.handle_waypoint(Frames.NED, 0, 0, -5.0, 0)
    

    # land
    drone.handle_landing()
    

if __name__ == '__main__':
    main()  # Calls main method if the python file is run directly (python3 filename.py)