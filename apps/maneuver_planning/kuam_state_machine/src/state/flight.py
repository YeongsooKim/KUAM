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
        # Initialize setpoint
        if self.setpoints.is_global:
            self.setpoint.is_global = True
        else:
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = True

        rospy.loginfo("flight Start is_global: %d", self.setpoints.is_global)

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
            self.setpoint.geopose, self.setpoint.pose = UpdateSetpoint(self.setpoints, self.ego_geopose, self.ego_pose, self.guidance_dist_th_m)
            
            # Check arrived
            if self.IsReached():
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
        msg.geopose = self.ego_geopose
        self.CompletionPub(msg)
        
        return trans


    def IsReached(self):
        if self.setpoint.is_global:
            dist = DistanceFromLatLonInMeter3D(self.setpoints.geopath.poses[-1].pose.position, self.ego_geopose.position)
            rospy.loginfo_throttle(0.1, "flight IsReached dist %f / sp: %f, %f, %f / ego: %f, %f, %f", dist, 
                            self.setpoints.geopath.poses[-1].pose.position.longitude, self.setpoints.geopath.poses[-1].pose.position.latitude, self.setpoints.geopath.poses[-1].pose.position.altitude, 
                            self.ego_geopose.position.longitude, self.ego_geopose.position.latitude, self.ego_geopose.position.altitude)
        else:
            dist = Distance3D(self.setpoints.pose_array.poses[-1].position, self.ego_pose.position)
            rospy.loginfo_throttle(0.1, "flight IsReached dist %f / sp: %f, %f, %f / ego: %f, %f, %f", dist, 
                            self.setpoints.pose_array.poses[-1].position.x, self.setpoints.pose_array.poses[-1].position.y, self.setpoints.pose_array.poses[-1].position.z, 
                            self.ego_pose.position.x, self.ego_pose.position.y, self.ego_pose.position.z)

        if dist < self.reached_dist_th_m:
            return True
        else: 
            return False

