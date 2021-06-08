import rospy
import smach
import state
import copy

class Arm(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints', 'ego_pose', 'ego_vel'], 
                                output_keys=['setpoint', 'setpoints', 'ego_pose', 'ego_vel'], 
                                outcomes=['disarm', 'takeoff', 'emerg', 'manual'])
        state.Base.__init__(self)

        self.transition = 'none'
  
    def execute(self, userdata):
        self.Start(userdata)
        self.Running()
        return self.Terminate(userdata)


    def Start(self, userdata):
        # Initialize setpoint
        self.setpoint = copy.deepcopy(userdata.setpoint)

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'disarm') or (self.transition == 'takeoff') or (self.transition == 'emerg') or (self.transition == 'manual'):
                    break
                else:
                    self.transition = 'none'  
                          
            # Update setpoint
            self.setpoint.pose = self.ego_pose
            rate.sleep()

    def Terminate(self, userdata):
        trans = self.transition
        userdata.setpoint = copy.deepcopy(self.setpoint)
        userdata.setpoints = copy.deepcopy(self.setpoints)
        userdata.ego_pose = copy.deepcopy(self.ego_pose)
        userdata.ego_vel = copy.deepcopy(self.ego_vel)

        self.transition = 'none'
        return trans