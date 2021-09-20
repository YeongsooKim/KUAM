import rospy
import smach
import state
import copy

class Arm(smach.State, state.Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['disarm', 'takeoff', 'emerg', 'manual', 'altctl'])
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
        self.is_start = True
        # Initialize setpoint
        pass

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'disarm') or (self.transition == 'takeoff') or \
                    (self.transition == 'emerg') or (self.transition == 'manual') or (self.transition == 'altctl'):
                    break
                else:
                    self.transition = 'none'  
                          
            # Update setpoint
            self.setpoint.geopose = copy.deepcopy(self.ego_geopose)
            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False
        return trans