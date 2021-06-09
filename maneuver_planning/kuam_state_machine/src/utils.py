from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
from math import pi
from math import sin
from math import cos
from math import atan2
from math import sqrt

EARTH_RADIUS_M = 6371e3

def Distance2D(point1, point2):
    delta_x = point1.x - point2.x
    delta_y = point1.y - point2.y
    
    distance_m = sqrt(pow(delta_x, 2) + pow(delta_y, 2))
    return distance_m
    
def Distance3D(point1, point2):
    delta_x = point1.x - point2.x
    delta_y = point1.y - point2.y
    delta_z = point1.z - point2.z
    
    distance_m = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2))
    return distance_m

def Deg2Rad(deg):
    rad = deg*pi/180.0
    
    return rad

def Rad2Deg(rad):
    deg = rad*180.0/pi

    return deg

# ref http://www.movable-type.co.uk/scripts/latlong.html
def DistanceFromLatLonInMeter2D(point1, point2):
    lat1 = point1.latitude
    lon1 = point1.longitude

    lat2 = point2.latitude
    lon2 = point2.longitude
    
    dLat = Deg2Rad(lat2 - lat1)
    dLon = Deg2Rad(lon2 - lon1)

    a = sin(dLat/2) * sin(dLat/2) + \
        cos(Deg2Rad(lat1)) * cos(Deg2Rad(lat2)) * \
        sin(dLon/2) * sin(dLon/2)

    c = 2*atan2(sqrt(a), sqrt(1 - a)); 
    distance_m = EARTH_RADIUS_M * c # Distance in m
    return distance_m

def DistanceFromLatLonInMeter3D(point1, point2):
    xy_d_m = DistanceFromLatLonInMeter2D(point1, point2)

    h_m = abs(point1.altitude - point2.altitude)

    distance_m = sqrt(pow(xy_d_m, 2) + pow(h_m, 2))
    return distance_m