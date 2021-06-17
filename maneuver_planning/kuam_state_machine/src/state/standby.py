import rospy
import smach
import state
import copy

class Standby(smach.State, state.Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['arm', 'emerg', 'manual'])
        state.Base.__init__(self)
        
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
        # Initialize setpoint
        pass
        
    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'arm') or (self.transition == 'emerg') or (self.transition == 'manual'):
                    break
                else:
                    self.transition = 'none'

            # Update setpoint
            self.setpoint.geopose = copy.deepcopy(self.ego_geopose)
            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        return trans