import rospy
import smach
import state
import copy

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils import *

class Takeoff(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints', 'ego_geopose', 'ego_pose', 'ego_vel'], 
                                output_keys=['setpoint', 'setpoints', 'ego_geopose', 'ego_pose', 'ego_vel'], 
                                outcomes=['done', 'emerg', 'manual'])
        state.Base.__init__(self)
        
        self.transition = 'none'
        self.takeoff_alt_m = None # defined by ros param
        self.reachied_dist_th_m = None # defined by ros param

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
        self.setpoint.geopose.position.altitude = self.takeoff_alt_m

    def Running(self):
        # wait for transition
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.transition != 'none':
                if (self.transition == 'done') or (self.transition == 'emerg') or (self.transition == 'manual'):        
                    break
                else:
                    self.transition = 'none'

            if self.IsReached():
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


    def IsReached(self):
        dist = DistanceFromLatLonInMeter3D(self.setpoint.geopose.position, self.ego_geopose.position)

        if (dist < self.reachied_dist_th_m):
            return True
        else: 
            return False

