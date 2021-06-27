import rospy
import smach
import state
import copy

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils import *

class Takeoff(smach.State, state.Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['done', 'emerg', 'manual'])
        state.Base.__init__(self)
        
        # Param
        self.takeoff_alt_m = None # defined by ros param
        self.reached_dist_th_m = None # defined by ros param

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

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False
        return trans


    def IsReached(self):
        dist = DistanceFromLatLonInMeter3D(self.setpoint.geopose.position, self.ego_geopose.position)

        if (dist < self.reached_dist_th_m):
            return True
        else: 
            return False

