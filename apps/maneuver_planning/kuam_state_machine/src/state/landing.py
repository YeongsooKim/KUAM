#!/usr/bin/env python3

import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import smach
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from .state import Base
from utils.util_geometry import *
from utils.util_state import *

from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from kuam_msgs.msg import LandingState
from geographic_msgs.msg import GeoPoseStamped
import tf2_geometry_msgs

class Landing(smach.State, Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['done', 'AUTO.RTL', 'MANUAL', 'ALTCTL'])
        Base.__init__(self)
        
        # Flag
        self.landing_state = LandingState()
        # self.landing_state.is_land = False
        
        # Param
        self.reached_dist_th_m = None # defined by ros param
        self.guidance_dist_th_m = None # defined by ros param
        self.landing_threshold_m = None
        self.using_camera = False
        self.maf_buf_size = 20
        self.big_target_mapping = 0.0
        self.medium_target_mapping = 0.0
        self.small_target_mapping = 0.0

        # State value
        self.ego_geopose = ego_geopose
        self.ego_pose = ego_pose
        self.ego_vel = ego_vel
        self.setpoint = setpoint
        self.setpoints = setpoints
        self.transition = 'none'
        
        # Target state
        self.target_id = 0 # Change to param
        self.target_pose = Pose()

        # tf
        self.tfBuffer = None
        self.listener = None

        self.standby_cnt = 0
        self.orientation = None

        '''
        Change target pose and target id to dictionary when the id is more than two
        '''
        
    def execute(self, userdata):
        self.Start()
        self.Running()
        return self.Terminate()

    '''
    State functions
    '''
    def Start(self):
        self.is_start = True
        
        self.switching_mod2 = False
        # Initialize setpoint
        # geopose = self.setpoints.poses[-1].pose
        # geopose.position.altitude = self.landing_standby_alt_m
        # self.landing_standby = geopose.position
        self.orientation = self.ego_geopose.orientation

        # in side the buffer: point, yaw
        self.maf_buf = [[Point(), 0.0]]*self.maf_buf_size
        # self.GenInitTrajectory(self.ego_pose, self.landing_standby)

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
            

            # # Update setpoint
            # self.UpdateSetpoint()

            # # Update landing state
            # self.UpdateLandingState()

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
        if not self.landing_state.is_detected:
            self.landing_state.mode = "standby"

        else:
            if -self.target_pose.position.z < self.landing_threshold_m:
                self.landing_state.mode = "auto.land"
                
            elif self.switching_mod2:
                self.landing_state.mode = "mode2"

            else:
                self.landing_state.mode = "mode1"

    def CameraSetpointUpdate(self):
        if self.landing_state.mode == "mode1":
            pass

        elif self.landing_state.mode == "mode2":
            self.setpoint.vel.linear.x = self.XY_Vel(self.target_pose.position.x)
            self.setpoint.vel.linear.y = self.XY_Vel(self.target_pose.position.y)
            self.setpoint.vel.linear.z = self.Z_Vel(self.target_pose.position.z)

            v_yaw = self.YawRate(self.target_pose.orientation)
            self.setpoint.yaw_rate.orientation.x = v_yaw[0]
            self.setpoint.yaw_rate.orientation.y = v_yaw[1]
            self.setpoint.yaw_rate.orientation.z = v_yaw[2]
            self.setpoint.yaw_rate.orientation.w = v_yaw[3]

            self.setpoint.geopose = self.ego_geopose

        elif self.landing_state.mode == "auto.land":
            self.transition = "done"

        elif self.landing_state.mode == "standby":
            self.setpoint.geopose = self.setpoints.poses[-1].pose
    
    '''
    GPS
    '''
    def GPSLandModeSwitching(self):
        if self.ego_geopose.position.altitude < self.landing_threshold_m:
            self.landing_state.mode = "auto.land"
        else:
            self.landing_state.mode = "mode1"
            

    def GPSSetpointUpdate(self):
        # Update setpoint
        if self.landing_state.mode == "mode1":
            self.setpoint.geopose = UpdateSetpointPose(self.setpoints.poses, self.ego_geopose, self.guidance_dist_th_m)
        elif self.landing_state.mode == "auto.land":
            self.transition = 'done'


    '''
    Process functions
    '''
    def GenInitTrajectory(self, ego_pose, landing_standby):
        self.standby_cnt = 0
        init_z = ego_pose.position.z
        target_alt = landing_standby.altitude
        target_lon = landing_standby.longitude
        target_lat = landing_standby.latitude
        target_vel_ms = 0.3

        interval = init_z - target_alt
        dt = 1.0/self.freq
        ds = target_vel_ms*dt
        n = int(interval/ds)

        del self.setpoints.poses[:]
        cnt = 0
        while cnt < n:
            geopose_stamped = GeoPoseStamped()

            geopose_stamped.header.frame_id = "map"
            geopose_stamped.header.stamp = rospy.Time.now()

            geopose_stamped.pose.position.latitude = target_lat
            geopose_stamped.pose.position.longitude = target_lon
            geopose_stamped.pose.position.altitude = init_z - ds*cnt

            geopose_stamped.pose.orientation = self.orientation

            self.setpoints.poses.append(geopose_stamped)
            cnt += 1

    def UpdateLandingState(self):
        if self.using_camera:
            if self.landing_state.is_detected:
                if -self.target_pose.position.z < self.landing_threshold_m:
                    # self.landing_state.is_land = True
                    pass
        else:
            if self.ego_geopose.position.altitude < self.landing_threshold_m:
                    # self.landing_state.is_land = True
                    pass

    def UpdateSetpoint(self):
        if self.using_camera:
            if self.landing_state.is_detected:
                self.setpoint.vel.linear.x = self.XY_Vel(self.target_pose.position.x)
                self.setpoint.vel.linear.y = self.XY_Vel(self.target_pose.position.y)
                self.setpoint.vel.linear.z = self.Z_Vel(self.target_pose.position.z)

                v_yaw = self.YawRate(self.target_pose.orientation)
                self.setpoint.yaw_rate.orientation.x = v_yaw[0]
                self.setpoint.yaw_rate.orientation.y = v_yaw[1]
                self.setpoint.yaw_rate.orientation.z = v_yaw[2]
                self.setpoint.yaw_rate.orientation.w = v_yaw[3]

                self.setpoint.pose = self.target_pose

        # only gps
        else :
            self.setpoint.vel.linear.x = 0.0
            self.setpoint.vel.linear.y = 0.0
            self.setpoint.vel.linear.z = -0.3
            self.setpoint.pose.orientation = self.orientation

    '''
    Util functions
    '''
    def IsValid(self, pose_stamped):
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y
        z = pose_stamped.pose.position.z

        if (x>1e+100) or (x<-1e+100):
            return False
        if (y>1e+100) or (y<-1e+100):
            return False
        if (z>1e+100) or (z<-1e+100):
            return False
        return True

    def Z_Vel(self, target):
        err = target
        kp = 0.045
        vel = err*kp

        th = 0.3
        if vel < -th:
            vel = -th
        elif vel > th:
            vel = th
        return vel

    def XY_Vel(self, target):
        err = target
        kp = 0.15
        vel = err*kp

        th = 0.5
        if vel < -th:
            vel = -th
        elif vel > th:
            vel = th
        return vel

    def YawRate(self, q):
        yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

        err = yaw_rad
        kp = 0.1

        yaw_rate_rad = err*kp

        vel = quaternion_from_euler(0.0, 0.0, yaw_rate_rad)
        return vel

    def MapTarget(self, pose, id):
        theta_rad = GetYawRad(pose.orientation)
        if id == 0:
            x_marker = -self.big_target_mapping
            y_marker = -self.big_target_mapping
        if id == 1:
            x_marker = +self.big_target_mapping
            y_marker = -self.big_target_mapping
        if id == 2:
            x_marker = +self.big_target_mapping
            y_marker = +self.big_target_mapping
        if id == 3:
            x_marker = -self.big_target_mapping
            y_marker = +self.big_target_mapping
        if id == 4:
            x_marker = -self.medium_target_mapping
            y_marker = 0.0
        if id == 5:
            x_marker = 0.0
            y_marker = -self.medium_target_mapping
        if id == 6:
            x_marker = +self.medium_target_mapping
            y_marker = 0.0
        if id == 7:
            x_marker = 0.0
            y_marker = +self.medium_target_mapping
        if id == 8:
            x_marker = 0.0
            y_marker = 0.0
        if id == 9:
            x_marker = -self.small_target_mapping
            y_marker = 0.0
        if id == 10:
            x_marker = 0.0
            y_marker = -self.small_target_mapping
        if id == 11:
            x_marker = +self.small_target_mapping
            y_marker = 0.0
        if id == 12:
            x_marker = 0.0
            y_marker = +self.small_target_mapping

        pose.position.x += self.X_Camera(x_marker, y_marker, theta_rad)
        pose.position.y += self.Y_Camera(x_marker, y_marker, theta_rad)
        return pose

    def X_Camera(self, x_marker, y_marker, theta_rad):
        x_camera = x_marker*cos(theta_rad) + y_marker*sin(theta_rad)
        return x_camera

    def Y_Camera(self, x_marker, y_marker, theta_rad):
        y_camera = x_marker*sin(theta_rad) - y_marker*cos(theta_rad)
        return y_camera

    def MovingAverageFilter(self, point, yaw_deg):
        # Initialize
        if not self.maf_init:
            for buf in self.maf_buf:
                buf[0].x = point.x
                buf[0].y = point.y
                buf[0].z = point.z
                buf[1] = yaw_deg

                self.maf_init = True
        
        # Shift
        if self.maf_buf_size != 1:
            for i in range(self.maf_buf_size - 1):
                self.maf_buf[i][0].x = self.maf_buf[i+1][0].x
                self.maf_buf[i][0].y = self.maf_buf[i+1][0].y
                self.maf_buf[i][0].z = self.maf_buf[i+1][0].z
                self.maf_buf[i][1] = self.maf_buf[i+1][1]
            self.maf_buf[self.maf_buf_size - 1][0].x = point.x
            self.maf_buf[self.maf_buf_size - 1][0].y = point.y
            self.maf_buf[self.maf_buf_size - 1][0].z = point.z
            self.maf_buf[self.maf_buf_size - 1][1] = yaw_deg
        else:
            self.maf_buf[0][0].x = point.x
            self.maf_buf[0][0].y = point.y
            self.maf_buf[0][0].z = point.z
            self.maf_buf[0][1] = yaw_deg

        # Summation
        sum_p = Point()
        sum_p.x = 0.0
        sum_p.y = 0.0
        sum_p.z = 0.0
        sum_y = 0.0
        for buf in self.maf_buf:
            sum_p.x += buf[0].x
            sum_p.y += buf[0].y
            sum_p.z += buf[0].z
            sum_y += buf[1]

        # Average
        ma_p = Point()
        ma_p.x = sum_p.x/self.maf_buf_size
        ma_p.y = sum_p.y/self.maf_buf_size
        ma_p.z = sum_p.z/self.maf_buf_size
        ma_y_rad = Deg2Rad(sum_y/self.maf_buf_size)
        q_ma_yaw = quaternion_from_euler(0.0, 0.0, ma_y_rad)

        self.target_pose.position.x = ma_p.x
        self.target_pose.position.y = ma_p.y
        self.target_pose.position.z = ma_p.z
        self.target_pose.orientation.x = q_ma_yaw[0]
        self.target_pose.orientation.y = q_ma_yaw[1]
        self.target_pose.orientation.z = q_ma_yaw[2]
        self.target_pose.orientation.w = q_ma_yaw[3]
        

    '''
    Callback functions
    '''
    def MarkerCB(self, msg):
        if msg.is_detected:
            self.landing_state.is_detected = True

            # find detected marker ids
            detected_marker_ids = []
            for aruco in msg.aruco_states:
                if aruco.is_detected:
                    detected_marker_ids.append(aruco.id)

            # compare detected big markers with used big markers, when all big markers are detected, mode switching to mode2
            self.switching_mod2 = True
            for id in msg.used_big_markers_id:
                if id not in detected_marker_ids:
                    self.switching_mod2 = False

            if not self.switching_mod2:
                return

            # transform target pose from camera_link to base_link
            try:
                transform = self.tfBuffer.lookup_transform('base_link', msg.header.frame_id, rospy.Time())
                p = PoseStamped()
                p.header = msg.header
                p.pose = msg.target_pose
                transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)

                self.target_pose = transformed_pose

            except:
                rospy.logerr("transform err")

        else:
            self.landing_state.is_detected = False
        
        # if not self.is_start:
        #     return

        # target_poses = []
        # for ac_state in msg.aruco_states:
        #     if not ac_state.is_detected:
        #         continue

        #     # transform from camera_link to base_link
        #     try:
        #         transform = self.tfBuffer.lookup_transform('base_link', ac_state.header.frame_id, rospy.Time())
        #         p = PoseStamped()
        #         p.header = ac_state.header
        #         p.pose = self.MapTarget(ac_state.pose, ac_state.id)
        #         transformed_pose = tf.do_transform_pose(p, transform)
        #         if self.IsValid(transformed_pose):
        #             target_poses.append(transformed_pose.pose)

        #         else:
        #             self.landing_state.is_detected = False
        #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        #         pass
        
        # if not len(target_poses) == 0:
        #     self.landing_state.is_detected = True

        #     sum_x = 0.0; sum_y = 0.0; sum_z = 0.0; sum_yaw_deg = 0.0
        #     for pose in target_poses:
        #         sum_x += pose.position.x
        #         sum_y += pose.position.y
        #         sum_z += pose.position.z
        #         sum_yaw_deg += GetYawDeg(pose.orientation)

        #     average_p = Point()

        #     average_p.x = sum_x/len(target_poses)
        #     average_p.y = sum_y/len(target_poses)
        #     average_p.z = sum_z/len(target_poses)
        #     avg_yaw_deg = sum_yaw_deg/len(target_poses)
            
        #     self.MovingAverageFilter(average_p, avg_yaw_deg)