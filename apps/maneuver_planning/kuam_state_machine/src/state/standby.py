import rospy
import smach
from .state import Base
import copy

class Standby(smach.State, Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['takeoff', 'AUTO.RTL', 'MANUAL', 'ALTCTL'])
        Base.__init__(self)
        
        # State value
        self.ego_geopose = ego_geopose
        self.ego_pose = ego_pose
        self.ego_vel = ego_vel
        self.setpoint = setpoint
        self.setpoints = setpoints
        self.transition = 'none'
        
    def execute(self, userdata):
        self.Start()
        self.Running()
        return self.Terminate()


    def Start(self):
        self.is_start = True
        # Initialize setpoint
        pass
        
    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'takeoff') or (self.transition == 'AUTO.RTL') or \
                    (self.transition == 'MANUAL') or (self.transition == 'ALTCTL'):
                    break
                else:
                    self.transition = 'none'

            # Update setpoint
            self.setpoint.geopose = copy.deepcopy(self.ego_geopose)
            self.setpoint.pose = copy.deepcopy(self.ego_pose)
            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False
        return trans