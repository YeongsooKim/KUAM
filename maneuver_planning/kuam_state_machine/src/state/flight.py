import rospy
import smach
import state
import copy
from kuam_msgs.msg import Setpoint

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils import *

class Flight(smach.State, state.Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['done', 'emerg', 'manual'])
        state.Base.__init__(self)

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
        # Initialize setpoint
        self.is_last = False
        self.is_first = True
        self.dest_geopose = copy.deepcopy(self.setpoints.poses[-1].pose)
        self.cur_sp_num = 0

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'done') or (self.transition == 'emerg') or (self.transition == 'manual'):        
                    break
                else:
                    self.transition = 'none'

            # Update setpoint
            if self.is_last == False:
                self.UpdateSetpointPose()
            else:
                if self.IsReached(self.ego_geopose.position, self.dest_geopose.position, self.reached_dist_th_m):
                    self.transition = 'done'

            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        return trans

    def UpdateSetpointPose(self):
        cur_sp_num = self.cur_sp_num
        cur_geopose = self.setpoints.poses[cur_sp_num].pose

        is_reached = self.IsReached(self.ego_geopose.position, cur_geopose.position, self.guidance_dist_th_m)

        if is_reached or self.is_first:
            if self.is_first:
                self.is_first = False

            if (cur_sp_num + 1) < len(self.setpoints.poses):
                self.cur_sp_num += 1
                self.is_last = False
                self.setpoint.geopose = self.setpoints.poses[self.cur_sp_num].pose
            else:
                self.is_last = True
                self.setpoint.geopose = self.dest_geopose

    def IsReached(self, p1, p2, th):
        dist = DistanceFromLatLonInMeter3D(p1, p2)
        if (dist < th):
            return True
        else: 
            return False