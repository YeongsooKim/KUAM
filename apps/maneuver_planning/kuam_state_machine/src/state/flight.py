import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import smach
import copy

from .state import Base
from utils.util_geometry import *
from utils.util_state import *

from kuam_msgs.msg import Setpoint
from kuam_msgs.msg import Completion



class Flight(smach.State, Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['done', 'AUTO.RTL', 'MANUAL', 'ALTCTL'])
        Base.__init__(self)

        # Publisher
        self.CompletionPub = None
        
        # Param
        self.reached_dist_th_m = None # defined by ros param
        self.guidance_dist_th_m = None # defined by ros param

        # Flag
        self.is_last = False
        self.is_first = True

        # State value
        self.ego_geopose = ego_geopose
        self.ego_pose = ego_pose
        self.ego_vel = ego_vel
        self.setpoint = setpoint
        self.setpoints = setpoints
        self.transition = 'none'
        self.cur_sp_num = 0

    def execute(self, userdata):

        self.Start()
        self.Running()
        return self.Terminate()


    def Start(self):
        pass

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'done') or (self.transition == 'emerg') or \
                    (self.transition == 'MANUAL') or (self.transition == 'ALTCTL'):
                    break
                else:
                    self.transition = 'none'

            # Update setpoint
            self.setpoint.geopose = UpdateSetpointPose(self.setpoints.poses, self.ego_geopose, self.guidance_dist_th_m)
            self.setpoint.is_global = True
            
            if IsReached(self.ego_geopose.position, self.setpoints.poses[-1].pose.position, self.reached_dist_th_m):
                self.transition = 'done'
                break

            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False

        msg = Completion()
        msg.task = "flight"
        msg.is_complete = not self.is_start
        msg.geopose = self.setpoints.poses[-1].pose
        self.CompletionPub(msg)
        
        return trans