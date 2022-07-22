from BasicArdu.BasicArdu import BasicArdu, Frames
from argparse import ArgumentParser
from time import sleep, time

def main():
    try:
        parser = ArgumentParser()
        parser.add_argument('--connection_string', type=str, default='/dev/ttyACM0', help='Ardupilot connection string')
        parser.add_argument('--file_name', type=str, default='data', help='log file name')
        options = parser.parse_args()

        # simple use example
        drone = BasicArdu(connection_string=options.connection_string, tolerance_location=1)    # connect to ArduPilot
        file = open(options.file_name+".txt", 'w')

        # takeoff drone
        drone.handle_takeoff(5)   
        sleep(3)
        start_time = time()
        while drone.vehicle.battery.voltage > 28.8:
            # goto first waypoint (5m north, 0 meters east, 5 meters up, facing North)
            drone.handle_waypoint(Frames.NED, 5, 0, -5, 0)
        
            # goto second wayoint(5m north, 5 meters east, 5 meters up, facing North)
            drone.handle_waypoint(Frames.NED, 5, 5, -5, 0)
            
            drone.handle_waypoint(Frames.NED, 0, 5, -5, 3.14)
            # sleep(3)
        
            # goto Home wayoint (0m north, 0 meters east, 5 meters up, facing North)
            drone.handle_waypoint(Frames.NED, 0, 0, -5, 0)
            sleep(3)
            print("Flight Time:", time()-start_time, "Battery:", drone.vehicle.battery.voltage)
            file.write("Flight Time:" + str(time()-start_time) + ", Battery: "+ str(drone.vehicle.battery.voltage)+"\n")

            

        # land
        drone.handle_landing()
        file.write("LANDED\n")
        print("LANDED")
        landed_time = time()
        while time()-landed_time < 60:
            print("Battery:", drone.vehicle.battery.voltage)
            file.write("Flight Time:" + str(time()-start_time) + ", Battery: "+ str(drone.vehicle.battery.voltage)+"\n")
            sleep(5)
        file.close()
        print("FINISHED")

    except Exception as e:
        print(e)
        file.close()
        # land
        drone.handle_landing()


        


if __name__ == '__main__':
    main()
