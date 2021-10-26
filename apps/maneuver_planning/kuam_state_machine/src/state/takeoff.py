import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import smach

from .state import Base
from utils.util_geometry import *

from kuam_msgs.msg import Completion


class Takeoff(smach.State, Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['done', 'AUTO.RTL', 'MANUAL', 'ALTCTL'])
        Base.__init__(self)

        # Publisher
        self.CompletionPub = None
        
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
        if self.setpoints.is_global:
            self.setpoint.is_global = True
        else:
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = True

        self.setpoint.geopose.position.altitude = self.takeoff_alt_m
        self.setpoint.pose.position.z = self.takeoff_alt_m

    def Running(self):
        # wait for transition
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            if self.transition != 'none':
                if (self.transition == 'done') or (self.transition == 'AUTO.RTL') or \
                    (self.transition == 'MANUAL') or (self.transition == 'ALTCTL'):
                    break
                else:
                    self.transition = 'none'

            if self.IsReached():
                self.transition = 'done'
                break
            rate.sleep()


    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False

        msg = Completion()
        msg.task = "takeoff"
        msg.is_complete = not self.is_start
        msg.geopose = self.setpoint.geopose
        self.CompletionPub(msg)

        return trans


    def IsReached(self):
        if self.setpoint.is_global:
            dist = DistanceFromLatLonInMeter3D(self.setpoint.geopose.position, self.ego_geopose.position)
        else:
            dist = Distance3D(self.setpoint.pose.position, self.ego_pose.position)

        if dist < self.reached_dist_th_m:
            return True
        else: 
            return False

