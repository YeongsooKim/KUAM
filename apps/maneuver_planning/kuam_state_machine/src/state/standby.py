import rospy
import smach
from .state import Base
import copy

from utils.util_state import *

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
        rospy.loginfo("## [standby] Start\n ##")

        # Initialize flag
        self.has_updated_setpoint = False
        self.is_start = True
        
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
            self.UpdateSetpoint()
            self.has_updated_setpoint = True

            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False
        return trans

    def UpdateSetpoint(self):
        rospy.loginfo_throttle(0.1, "### [standby] UpdateSetpoint ###")

        self.setpoint.geopose = copy.deepcopy(self.ego_geopose)
        self.setpoint.pose = copy.deepcopy(self.ego_pose)

        if self.setpoints.is_global:
            self.setpoint.is_global = True
            rospy.loginfo_throttle(0.1, "[standby] setpoint: global\n")

        else:
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = False
            rospy.loginfo_throttle(0.1, "[standby] setpoint: setpoint_raw\n")

            self.setpoint.vel.linear.x = 0.0
            self.setpoint.vel.linear.y = 0.0
            self.setpoint.vel.linear.z = 0.0

            v_yaw = YawRateRad(0.0, 0.1)
            self.setpoint.yaw_rate.orientation.x = v_yaw[0]
            self.setpoint.yaw_rate.orientation.y = v_yaw[1]
            self.setpoint.yaw_rate.orientation.z = v_yaw[2]
            self.setpoint.yaw_rate.orientation.w = v_yaw[3]