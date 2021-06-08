from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
import math
from math import pi


def Distance2D(point1, point2):
    delta_x = point1.x - point2.x
    delta_y = point1.y - point2.y
    
    distance_m = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
    return distance_m
    
def Distance3D(point1, point2):
    delta_x = point1.x - point2.x
    delta_y = point1.y - point2.y
    delta_z = point1.z - point2.z
    
    distance_m = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2))
    return distance_m

def Deg2Rad(deg):
    rad = deg*pi/180.0
    
    return rad

def Rad2Deg(rad):
    deg = rad*180.0/pi

    return deg