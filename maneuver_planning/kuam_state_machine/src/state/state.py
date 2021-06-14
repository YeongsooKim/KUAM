from kuam_msgs.msg import Setpoint
from geometry_msgs.msg import Twist
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import Pose

class Base():
    def __init__(self):
        self.transition = ''
        self.freq = 1.0

        self.ego_geopose = GeoPose()
        self.ego_pose = Pose()
        self.ego_vel = Twist()

        self.setpoint = Setpoint()
        self.setpoints = GeoPath()
        
    def IsGeoPathEmpty(self):
        if (len(self.geo_setpoints.poses) == 0):
            return True
        else:
            return False