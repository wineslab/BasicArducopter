import enum
import math

class Frames(enum.Enum):
    ''' 
    Describes the coordinate frame that a waypoint is in
    '''
    LLA = 'LLA'
    NED = 'NED'
    VEL = 'VEL'


R = 6378137.0  # Equator radius in meters


def arc_to_deg(arc):
    """convert spherical arc length [m] to great circle distance [deg]"""
    return float(arc)/6371/1000 * 180/math.pi

def deg_to_arc(deg):
    """convert great circle distance [deg] to spherical arc length [m]"""
    return float(deg)*6371*1000 * math.pi/180

def latlon_to_xyz(lat,lon):
    """Convert angluar to cartesian coordiantes

    latitude is the 90deg - zenith angle in range [-90;90]
    lonitude is the azimuthal angle in range [-180;180] 
    """
    r = 6371 
    theta = math.pi/2 - math.radians(lat) 
    phi = math.radians(lon)
    x = r * math.sin(theta) * math.cos(phi) 
    y = r * math.sin(theta) * math.sin(phi)
    z = r * math.cos(theta)
    return [x,y,z]

def xyz_to_latlon (ref, dn, de, dd):
    """
    Convert cartesian to angular lat/lon coordiantes
    :param ref: Waypoint object for reference origin
    :param dn: float meters north
    :param de: float meters east
    :param dd: float meters down
    :return: Waypoint class object of the desired position in LLA
    """
    dLat = dn/R
    dLon = de/(R*math.cos(math.pi*ref.lat/180))

    lat = ref.lat + (dLat*180/math.pi)
    lon = ref.lon + (dLon*180/math.pi)
    alt = ref.alt - dd

    return Waypoint(x=lat, y=lon, z=alt)

#######################################
### Waypoint Object
#######################################

class Waypoint():
    def __init__(self, x=0.0, y=0.0, z=0.0, compass_angle=0.0, name=None):
        '''
        Waypoint Class
        :param x: float, lat for LLA
        :param y: float, long for LLA
        :param z: alt msl for LLA
        :param compass_angle: float for  the desired yaw angle (0 = North, pi/2 = east)
        :param name: string of the name of the waypoint
        '''
        self.name = name            # Name, currently unused
        self.phi = compass_angle    # Desired compass angle at waypoint (0 = North, 90 = East) default: 0

        self.lat = x
        self.lon = y
        self.alt = z

    def update (self, vehicle):
        '''
        Method will update the waypoint with the current vehicle location
        :param vehicle: vehicle dronekit object
        '''
        self.lat = vehicle.location.global_frame.lat
        self.lon = vehicle.location.global_frame.lon
        self.alt = vehicle.location.global_frame.alt	
    
    def current_distance(self, vehicle, to_waypoint=None):
        '''
        Method to calculate the distance between waypoints. If two_waypoint is None, then distance from drone to waypoint is used
        :param vehicle: Dronekit vehicle object
        :param to_waypoint: is a Waypoint object. If inputted, vehicle is ignored
        :return: the current distance in meters from the vehicle to the waypoint or an inputted waypoint
        '''

        if to_waypoint == None: # If no waypoint is specified, give distance from vehicle
            dlat = self.lat - vehicle.location.global_frame.lat
            dlong = self.lon - vehicle.location.global_frame.lon
            dalt = self.alt - vehicle.location.global_frame.alt
            phi_2=math.radians(vehicle.location.global_frame.lat)
        else:
            dlat = self.lat - to_waypoint.lat
            dlong = self.lon - to_waypoint.lon
            dalt = self.alt - to_waypoint.alt
            phi_2=math.radians(to_waypoint.lat)
        
        #R=6371000                               # radius of Earth in meters
        phi_1=math.radians(self.lat)
        

        delta_phi=math.radians(dlat)
        delta_lambda=math.radians(dlong)

        a=math.sin(delta_phi/2.0)**2+\
            math.cos(phi_1)*math.cos(phi_2)*\
            math.sin(delta_lambda/2.0)**2

        c=2*math.atan2(math.sqrt(a),math.sqrt(1-a))

        ground_dist = R*c 

        dist = math.sqrt((ground_dist * ground_dist) + (dalt * dalt))   # add altitude distance
        return dist                        # output distance in meters
        

    def current_bearing(self, vehicle, to_waypoint=None):
        '''
        Method to calculate the bearing between two lat/lon waypoints
        :param vehicle: Dronekit vehicle object
        :param to_waypoint: is a Waypoint object. If inputted, vehicle is ignored
        :return: float of the bearing in radians
        '''
        if to_waypoint==None:   # if no waypoint specified, then give vehicle bearing
            lat1 = vehicle.location.global_frame.lat
            lon1 = vehicle.location.global_frame.lon
        else:
            lat1 = to_waypoint.lat
            lon1 = to_waypoint.lon
            
        lat2 = self.lat
        lon2 = self.lon

        x1, y1, z1 = latlon_to_xyz(lat1, lon1)
        x2, y2, z2 = latlon_to_xyz(lat2, lon2)
        dx = x2 - x1
        dy = y2 - y1
        theta = math.atan2(dy, dx)
        theta = 2*math.pi - theta + math.pi/2
        return theta 
    
    def LLA_2_Coords(self, vehicle, to_waypoint=None):
        '''
        Method to calculate the approximated dNorth and dEast coordinates of a waypoint realative to this waypoint
        :param vehicle: Dronekit vehicle object
        :param to_waypoint: is a Waypoint object. If inputted, vehicle is ignored
        :return: array of floats [dNorth, dEast] in meters
        '''
        f = 0.00335281066474748071  # 1/298.257223563, inverse flattening

        if to_waypoint==None:   # if no waypoint specified, then give vehicle bearing
            lat1 = vehicle.location.global_frame.lat
            lon1 = vehicle.location.global_frame.lon
        else:
            lat1 = to_waypoint.lat
            lon1 = to_waypoint.lon
            
        Lat_p = lat1 * math.pi / 180.0  # from degrees to radians
        Lon_p = lon1 * math.pi / 180.0  # from degrees to radians

        # Reference location (lat, lon), from degrees to radians
        Lat_o = self.lat * math.pi / 180.0
        Lon_o = self.lon * math.pi / 180.0
        
        dLat = Lat_p - Lat_o
        dLon = Lon_p - Lon_o

        ff = (2.0 * f) - (f ** 2)  # Can be precomputed
        sinLat = math.sin(Lat_o)
        # Radius of curvature in the prime vertical
        Rn = R / math.sqrt(1 - (ff * (sinLat ** 2)))

        # Radius of curvature in the meridian
        Rm = Rn * ((1 - ff) / (1 - (ff * (sinLat ** 2))))

        dNorth = (dLat) / math.atan2(1, Rm)
        dEast = (dLon) / math.atan2(1, (Rn * math.cos(Lat_o)))
        return [dNorth, dEast]

    def print(self):
        print("lat : {} lon: {} alt: {} phi: {}".format(self.lat, self.lon, self.alt, self.phi))
