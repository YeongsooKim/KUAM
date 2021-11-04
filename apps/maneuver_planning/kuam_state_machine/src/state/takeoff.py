import os
import copy
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import smach

from .state import Base
from utils.util_geometry import *
from utils.util_state import *

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
        rospy.loginfo("## [takeoff] Start ##")

        # Initialize flag
        self.has_updated_setpoint = False
        self.is_start = True

        # Initialize setpoint
        self.takeoff_geopose = copy.deepcopy(self.ego_geopose)
        self.takeoff_geopose.position.altitude += self.takeoff_alt_m
        self.takeoff_pose = copy.deepcopy(self.ego_pose)
        self.takeoff_pose.position.z += self.takeoff_alt_m

        rospy.loginfo("[takeoff] Start global sp: %f, %f, %f / local sp: %f, %f, %f\n",
                    self.takeoff_geopose.position.longitude, self.takeoff_geopose.position.latitude, self.takeoff_geopose.position.altitude,
                    self.takeoff_pose.position.x, self.takeoff_pose.position.y, self.takeoff_pose.position.z)


    def Running(self):
        # wait for transition
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'done') or (self.transition == 'AUTO.RTL') or \
                    (self.transition == 'MANUAL') or (self.transition == 'ALTCTL'):
                    break
                else:
                    self.transition = 'none'

            # Update setpoint
            self.UpdateSetpoint()
            self.has_updated_setpoint = True

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
        msg.task = "takeoff"
        msg.is_complete = not self.is_start
        msg.geopose = self.ego_geopose
        self.CompletionPub(msg)

        return trans


    def IsReached(self):
        rospy.loginfo_throttle(0.1, "### [takeoff] IsReached ###")

        if self.setpoint.is_global:
            dist = abs(self.setpoint.geopose.position.altitude - self.ego_geopose.position.z)
            rospy.loginfo_throttle(0.1, "[takeoff] IsReached dist %f / sp: %f, %f, %f / ego: %f, %f, %f\n", dist, 
                            self.setpoint.geopose.position.longitude, self.setpoint.geopose.position.latitude, self.setpoint.geopose.position.altitude, 
                            self.ego_geopose.position.longitude, self.ego_geopose.position.latitude, self.ego_geopose.position.altitude)
        else:
            dist = abs(self.setpoint.pose.position.z - self.ego_pose.position.z)
            rospy.loginfo_throttle(0.1, "[takeoff] IsReached dist %f / sp: %f, %f, %f / ego: %f, %f, %f\n", dist, 
                            self.setpoint.pose.position.x, self.setpoint.pose.position.y, self.setpoint.pose.position.z, 
                            self.ego_pose.position.x, self.ego_pose.position.y, self.ego_pose.position.z)

        if dist < self.reached_dist_th_m:
            return True
        else: 
            return False


    def UpdateSetpoint(self):
        rospy.loginfo_throttle(0.1, "### [takeoff] UpdateSetpoint ###")

        if self.setpoints.is_global:
            self.setpoint.is_global = True
            rospy.loginfo_throttle(0.1, "[takeoff] setpoint: global\n")

            self.setpoint.geopose = self.takeoff_geopose
            self.setpoint.pose = self.takeoff_pose
        else:
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = False
            rospy.loginfo_throttle(0.1, "[takeoff] setpoint: setpoint_raw")

            self.setpoint.vel.linear.x = XY_Vel(0.0, 1.0, 1.2)
            self.setpoint.vel.linear.y = XY_Vel(0.0, 1.0, 1.2)

            err = self.takeoff_pose.position.z - self.ego_pose.position.z
            self.setpoint.vel.linear.z = Z_Vel(err, 0.9, 1.5)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f\n", err, self.setpoint.vel.linear.z)

            v_yaw = YawRateRad(0.0, 0.1)
            self.setpoint.yaw_rate.orientation.x = v_yaw[0]
            self.setpoint.yaw_rate.orientation.y = v_yaw[1]
            self.setpoint.yaw_rate.orientation.z = v_yaw[2]
            self.setpoint.yaw_rate.orientation.w = v_yaw[3]

            self.setpoint.geopose = self.takeoff_geopose
            self.setpoint.pose = self.takeoff_pose