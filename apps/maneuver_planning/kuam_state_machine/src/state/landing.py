#!/usr/bin/env python3

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import smach

from .state import Base
from utils.util_geometry import *
from utils.util_state import *

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from kuam_msgs.msg import LandingState
from geographic_msgs.msg import GeoPoseStamped
import tf2_geometry_msgs
import tf2_ros

class Landing(smach.State, Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['done', 'AUTO.RTL', 'MANUAL', 'ALTCTL'])
        Base.__init__(self)
        
        # Flag
        self.switching_mod2 = False
        self.switching_mod1 = False
        self.is_mod2_fixed = False
        
        # Param
        self.reached_dist_th_m = None # defined by ros param
        self.roi_th_m = None # defined by ros param
        self.landing_threshold_m = None
        self.using_camera = False

        # State value
        self.ego_geopose = ego_geopose
        self.ego_pose = ego_pose
        self.ego_vel = ego_vel
        self.setpoint = setpoint
        self.setpoints = setpoints
        self.transition = 'none'
        self.prev_global_nearest_sp_idx = 0
        self.prev_local_nearest_sp_idx = 0
        
        # Target state
        self.marker_pose = Pose()
        self.vehicle_pose = Pose()

        # Tf
        self.tfBuffer = None
        self.listener = None
        
    def execute(self, userdata):
        self.Start()
        self.Running()
        return self.Terminate()

    '''
    State functions
    '''
    def Start(self):
        rospy.loginfo("## [landing] Start\n ##")

        # Initialize flag
        self.has_updated_setpoint = False
        self.is_start = True
        self.switching_mod2 = False
        self.switching_mod1 = False
        self.is_mod2_fixed = False

        self.prev_global_nearest_sp_idx = 0
        self.prev_local_nearest_sp_idx = 0

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'done') or (self.transition == 'AUTO.RTL') or \
                    (self.transition == 'MANUAL') or (self.transition == 'ALTCTL'):
                    break
                else:
                    self.transition = 'none'    

            # Using camera
            if self.using_camera:
                self.CameraLandModeSwitching()
                self.CameraSetpointUpdate()

            # Using GPS only
            else:
                self.GPSLandModeSwitching()
                self.GPSSetpointUpdate()

            self.has_updated_setpoint = True
            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False
        return trans

    '''
    Camera
    '''
    def CameraLandModeSwitching(self):
        if -self.marker_pose.position.z < self.landing_threshold_m and -self.marker_pose.position.z > 0.01:
            self.mode = "auto.land"
            
        elif self.switching_mod2:
            self.mode = "mode2"

        elif self.switching_mod1:
            self.mode = "mode1"
        else:
            self.mode = "standby"
        
    def CameraSetpointUpdate(self):
        rospy.loginfo_throttle(0.1, "### [landing] CameraSetpointUpdate ###")

        landing_state = LandingState()
        landing_state.mode = self.mode

        if self.mode == "mode1":
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = False
            rospy.loginfo_throttle(0.1, "[landing] mode1 setpoint: setpoint_raw")

            self.setpoint.vel.linear.x = XY_Vel(self.vehicle_pose.position.x, 0.15, 0.3)
            self.setpoint.vel.linear.y = XY_Vel(self.vehicle_pose.position.y, 0.15, 0.3)
            self.setpoint.vel.linear.z = Z_Vel(self.vehicle_pose.position.z, 0.045, 0.5)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f", self.vehicle_pose.position.x, self.setpoint.vel.linear.x)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f", self.vehicle_pose.position.y, self.setpoint.vel.linear.y)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f\n", self.vehicle_pose.position.z, self.setpoint.vel.linear.z)

            v_yaw = YawRate(self.setpoints.pose_array.poses[-1].orientation, 0.1)
            self.setpoint.yaw_rate.orientation.x = v_yaw[0]
            self.setpoint.yaw_rate.orientation.y = v_yaw[1]
            self.setpoint.yaw_rate.orientation.z = v_yaw[2]
            self.setpoint.yaw_rate.orientation.w = v_yaw[3]

            self.setpoint.ego_pose = self.ego_pose
            self.setpoint.target_pose = self.vehicle_pose

            landing_state.is_vehicle_detected = True

        elif self.mode == "mode2":
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = False
            rospy.loginfo_throttle(0.1, "[landing] mode2 setpoint: setpoint_raw")

            self.setpoint.vel.linear.x = XY_Vel(self.marker_pose.position.x, 0.15, 0.3)
            self.setpoint.vel.linear.y = XY_Vel(self.marker_pose.position.y, 0.15, 0.3)
            self.setpoint.vel.linear.z = Z_Vel(self.marker_pose.position.z, 0.045, 0.5)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f", self.marker_pose.position.x, self.setpoint.vel.linear.x)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f", self.marker_pose.position.y, self.setpoint.vel.linear.y)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f\n", self.marker_pose.position.z, self.setpoint.vel.linear.z)

            v_yaw = YawRate(self.marker_pose.orientation, 0.1)
            self.setpoint.yaw_rate.orientation.x = v_yaw[0]
            self.setpoint.yaw_rate.orientation.y = v_yaw[1]
            self.setpoint.yaw_rate.orientation.z = v_yaw[2]
            self.setpoint.yaw_rate.orientation.w = v_yaw[3]

            self.setpoint.ego_pose = self.ego_pose
            self.setpoint.target_pose = self.marker_pose

            landing_state.is_marker_detected = True
            if self.switching_mod1 == True:
                landing_state.is_vehicle_detected = True

        elif self.mode == "auto.land":
            self.setpoint.is_global = self.setpoints.is_global
            rospy.loginfo_throttle(0.1, "[landing] auto.land setpoint: global\n")

            self.transition = 'done'

        elif self.mode == "standby":
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = False
            rospy.loginfo_throttle(0.1, "[landing] standby setpoint: setpoint_raw")

            self.setpoint.vel.linear.x = XY_Vel(0.0, 0.15, 0.3)
            self.setpoint.vel.linear.y = XY_Vel(0.0, 0.15, 0.3)
            self.setpoint.vel.linear.z = Z_Vel(0.0, 0.045, 0.5)
            rospy.loginfo_throttle(0.1, "err: 0.0, vel: %f", self.setpoint.vel.linear.x)
            rospy.loginfo_throttle(0.1, "err: 0.0, vel: %f", self.setpoint.vel.linear.y)
            rospy.loginfo_throttle(0.1, "err: 0.0, vel: %f\n", self.setpoint.vel.linear.z)

            v_yaw = YawRateRad(0.0, 0.1)
            self.setpoint.yaw_rate.orientation.x = v_yaw[0]
            self.setpoint.yaw_rate.orientation.y = v_yaw[1]
            self.setpoint.yaw_rate.orientation.z = v_yaw[2]
            self.setpoint.yaw_rate.orientation.w = v_yaw[3]

            self.setpoint.ego_pose = self.ego_pose
            self.setpoint.target_pose = self.vehicle_pose

        self.setpoint.landing_state = landing_state
    
    '''
    GPS
    '''
    def GPSLandModeSwitching(self):
        if self.ego_geopose.position.altitude < self.landing_threshold_m:
            self.mode = "auto.land"
        else:
            self.mode = "mode1"
            

    def GPSSetpointUpdate(self):
        rospy.loginfo_throttle(0.1, "### [landing] GPSSetpointUpdate ###")

        # Update setpoint
        landing_state = LandingState()
        landing_state.mode = self.mode

        if self.mode == "mode1":
            # Update setpoint
            self.setpoint.is_global = True
            rospy.loginfo_throttle(0.1, "[landing] mode1 setpoint: global\n")

            optimal_setpoint = GetOptimalSetpoint(self.setpoints, self.ego_geopose, self.ego_pose, 
                                        self.roi_th_m, self.prev_global_nearest_sp_idx, self.prev_local_nearest_sp_idx)
            self.setpoint.geopose = optimal_setpoint[0]
            self.prev_global_nearest_sp_idx = optimal_setpoint[1]
            self.setpoint.pose = optimal_setpoint[2]
            self.prev_local_nearest_sp_idx = optimal_setpoint[3]
            rospy.loginfo_throttle(0.1, "global sp: %f, %f, %f", self.setpoint.geopose.position.longitude, self.setpoint.geopose.position.latitude, self.setpoint.geopose.position.altitude)
            rospy.loginfo_throttle(0.1, "global ego: %f, %f, %f", self.ego_geopose.position.longitude, self.ego_geopose.position.latitude, self.ego_geopose.position.altitude)

            
        elif self.mode == "auto.land":
            self.setpoint.is_global = True
            rospy.loginfo_throttle(0.1, "[landing] auto.land setpoint: global\n")

            self.transition = 'done'

        self.setpoint.landing_state = landing_state


    '''
    Callback functions
    '''
    def MarkerCB(self, msg):
        if not msg.is_detected:
            return

        # compare detected big markers with used big markers, when all big markers are detected, mode switching to mode2
        if not self.is_mod2_fixed:
            if msg.is_detected:
                self.switching_mod2 = True

            if not self.switching_mod2:
                return
            else:
                self.is_mod2_fixed = True

        # transform target pose from camera_link to base_link
        try:
            transform = self.tfBuffer.lookup_transform('base_link', msg.header.frame_id, rospy.Time())
            p = PoseStamped()
            p.header = msg.header
            p.pose = msg.target_pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)

            self.marker_pose = transformed_pose.pose

        except:
            rospy.logerr("[landing state] transform err")

    def VehicleCB(self, msg):
        if not msg.is_detected:
            return

        # mode change
        self.switching_mod1 = True

        # transform vehicle pixel from camera_link to base_link
        try:
            transform = self.tfBuffer.lookup_transform('base_link', msg.header.frame_id, rospy.Time())
            p = PoseStamped()
            p.header = msg.header
            p.pose = msg.vehicle
            transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)

            self.vehicle_pose = transformed_pose.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("[landing state] transform err. msg frame_id: %s", msg.header.frame_id)