from geometry_msgs.msg import Pose
from kuam_msgs.msg import LandingState
import rospy
import smach
import state
import copy

class Landing(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints'], 
                                output_keys=['setpoint', 'setpoints'], 
                                outcomes=['emerg', 'manual'])
        state.Base.__init__(self)

        self.transition = 'none'
        self.is_detected = False
        self.is_land = False
        self.target_id = 0 # Change to param
        self.target_pose = Pose()
        self.landing_state_pub = None
        self.landing_threshold_m = None
        self.landing_standby_alt_m = 5.0

        '''
        Change target pose and target id to dictionary when the id is more than two
        '''
        
    def execute(self, userdata):
        self.Start(userdata)
        self.Running()
        return self.Terminate(userdata)


    def Start(self, userdata):
        # Initialize setpoint
        self.setpoints = copy.deepcopy(userdata.setpoints)
        self.setpoint.pose = self.setpoints.poses[-1]
        self.setpoint.pose.position.z = self.landing_standby_alt_m

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'emerg') or (self.transition == 'manual'):
                    break
                else:
                    self.transition = 'none'    
            # Update virtual border

            # Update setpoint
            if self.is_detected == True:
                self.setpoint.pose = self.target_pose

            # Publish
            self.LandingStatePub()

            rate.sleep()

    def Terminate(self, userdata):
        trans = self.transition

        self.transition = 'none'
        return trans

    def LandingStatePub(self):
        if self.landing_threshold_m is None:
            pass
        else:
            h = self.ego_pose.position.z

            if h < self.landing_threshold_m:
                self.is_land = True

            landing_state_msg = LandingState()
            landing_state_msg.is_detected = self.is_detected
            landing_state_msg.is_land = self.is_land
            self.landing_state_pub.publish(landing_state_msg)

    # # Get virtual border side
    # def GetVBside(self):
    #     '''
    #     ego orientation -> roll, pitch, yaw 
    #     get VBS (virtual border side).
    #     body -> global (trans, yaw)
    #     '''