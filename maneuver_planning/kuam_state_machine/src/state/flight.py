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
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints', 'ego_geopose', 'ego_pose', 'ego_vel'], 
                                output_keys=['setpoint', 'setpoints', 'ego_geopose', 'ego_pose', 'ego_vel'], 
                                outcomes=['done', 'emerg', 'manual'])
        state.Base.__init__(self)
        
        # Param
        self.reachied_dist_th_m = None # defined by ros param
        self.guidance_dist_th_m = None # defined by ros param

        # Flag
        self.is_last = False

        self.transition = 'none'
        self.cur_sp_num = 0

    def execute(self, userdata):

        self.Start(userdata)
        self.Running()
        return self.Terminate(userdata)


    def Start(self, userdata):
        # Initialize setpoint
        self.setpoint = copy.deepcopy(userdata.setpoint)
        self.setpoints = copy.deepcopy(userdata.setpoints)
        self.ego_pose = copy.deepcopy(userdata.ego_pose)
        self.ego_geopose = copy.deepcopy(userdata.ego_geopose)
        self.dest_geopose = self.setpoints.poses[-1].pose
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
                self.setpoint.geopose = self.NextSetpointPose()
            else:
                if self.IsReached(self.ego_geopose.position, self.dest_geopose.position, self.reachied_dist_th_m):
                    self.transition = 'done'
                    
            rate.sleep()

    def Terminate(self, userdata):
        trans = self.transition
        userdata.setpoint = copy.deepcopy(self.setpoint)
        userdata.setpoints = copy.deepcopy(self.setpoints)
        userdata.ego_pose = copy.deepcopy(self.ego_pose)
        userdata.ego_geopose = copy.deepcopy(self.ego_geopose)
        userdata.ego_vel = copy.deepcopy(self.ego_vel)

        self.transition = 'none'
        return trans
            
    def NextSetpointPose(self):
        cur_sp_num = self.cur_sp_num
        cur_geopose = copy.deepcopy(self.setpoints.poses[cur_sp_num].pose)

        is_reached = self.IsReached(self.ego_geopose.position, cur_geopose.position, self.guidance_dist_th_m)
        self.is_last = False

        if is_reached == True:
            cur_sp_num += 1

            if cur_sp_num == len(self.setpoints.poses):
                self.is_last = True
                
                return self.dest_geopose
            else:
                self.cur_sp_num = cur_sp_num

        return self.setpoints.poses[self.cur_sp_num].pose


    def IsReached(self, p1, p2, th):
        dist = DistanceFromLatLonInMeter3D(p1, p2)
        if (dist < th):
            return True
        else: 
            return False