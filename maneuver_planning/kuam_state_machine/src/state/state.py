from kuam_msgs.msg import Setpoint
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

class Base():
    def __init__(self):
        self.transition = ''
        self.freq = 1.0

        self.ego_pose = Pose()

        self.setpoint = Setpoint()
        self.setpoints = PoseArray()
        
    def IsGeoPathEmpty(self):
        if (len(self.geo_setpoints.poses) == 0):
            return True
        else:
            return False