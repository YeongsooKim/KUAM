from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoint
import numpy as np
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

def GetYawRad(orientation):
    euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    yaw_rad = euler[2]

    return yaw_rad

def GetYawDeg(orientation):
    euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    yaw_rad = euler[2]
    yaw_deg = yaw_rad*180.0/pi

    return yaw_deg

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


def LengthOfDegreeLatitude(ref_latitude_deg):
    # Convert latitude to radians
    ref_latitude_rad = float(ref_latitude_deg) * np.pi / 180.

    # Set up "Constants"
    m1 = 111132.92 # Latitude calculation term 1
    m2 = -559.82 # Latitude calculation term 2
    m3 = 1.175 # Latitude calculation term 3
    m4 = -0.0023 # Latitude calculation term 4

    # Calculate the length of a degree of latitude and longitude in meters
    return m1 + (m2 * np.cos(2 * ref_latitude_rad)) + (m3 * np.cos(4 * ref_latitude_rad)) + (m4 * np.cos(6 * ref_latitude_rad))
    
def LengthOfDegreeLongitude(ref_latitude_deg):
    # Convert latitude to radians
    ref_latitude_rad = float(ref_latitude_deg) * np.pi / 180.

    # Set up "Constants"
    p1 = 111412.84 # Longitude calculation term 1
    p2 = -93.5 # Longitude calculation term 2
    p3 = 0.118 # Longitude calculation term 3

    # Calculate the length of a degree of latitude and longitude in meters
    return (p1 * np.cos(ref_latitude_rad)) + (p2 * np.cos(3 * ref_latitude_rad)) + (p3 * np.cos(5 * ref_latitude_rad))
    
def CartesianToWgs84( geopoint_ref, enupoint ):
    # geopoint_ref: [latitude, longitude] list for origin points
    # enupoint: [east, north] list        
    latitude = geopoint_ref[0] + enupoint[1]/LengthOfDegreeLatitude(geopoint_ref[0])
    longitude = geopoint_ref[1] + enupoint[0]/LengthOfDegreeLongitude(geopoint_ref[0])
    return [latitude, longitude]

def Wgs84ToCartesian( geopoint_ref, wgspoint ):
    east = (wgspoint[1] - geopoint_ref[1]) * LengthOfDegreeLongitude(geopoint_ref[0])
    north = (wgspoint[0] - geopoint_ref[0]) * LengthOfDegreeLatitude(geopoint_ref[0])
        
    return [east, north]
