import rospy
import smach
import copy
import state

class Hovering(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints', 'ego_pose', 'ego_vel'], 
                                output_keys=['setpoint', 'setpoints', 'ego_pose', 'ego_vel'], 
                                outcomes=['flight', 'landing', 'emerg', 'manual'])
        state.Base.__init__(self)
        
        self.transition = 'none'
        
    def execute(self, userdata):
        self.Start(userdata)
        self.Running()
        return self.Terminate(userdata)


    def Start(self, userdata):
        # Initialize setpoint
        self.setpoint = copy.deepcopy(userdata.setpoint)
        self.setpoints = copy.deepcopy(userdata.setpoints)

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'flight') or (self.transition == 'landing') or (self.transition == 'emerg') or (self.transition == 'manual'):
                    break
                else:
                    self.transition = 'none'        
            rate.sleep()

    def Terminate(self, userdata):
        trans = self.transition
        userdata.setpoint = copy.deepcopy(self.setpoint)
        userdata.setpoints = copy.deepcopy(self.setpoints)
        userdata.ego_pose = copy.deepcopy(self.ego_pose)
        userdata.ego_vel = copy.deepcopy(self.ego_vel)

        self.transition = 'none'
        return trans