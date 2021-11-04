import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

import rospy
import smach
import copy
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from .state import Base
from utils.util_geometry import *
from utils.util_state import *

from kuam_msgs.msg import Setpoint
from kuam_msgs.msg import Completion

from geometry_msgs.msg import PoseStamped
import tf2_geometry_msgs


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
        self.roi_th_m = None # defined by ros param

        # Flag
        self.is_last = False
        self.is_first = True
        self.landing_switching_mod2 = False
        self.landing_switching_mod1 = False

        # Tf
        self.tfBuffer = None
        self.listener = None
        
        # State value
        self.ego_geopose = ego_geopose
        self.ego_pose = ego_pose
        self.ego_vel = ego_vel
        self.setpoint = setpoint
        self.setpoints = setpoints
        self.transition = 'none'
        self.cur_sp_num = 0
        self.prev_global_nearest_sp_idx = 0
        self.prev_local_nearest_sp_idx = 0

    def execute(self, userdata):

        self.Start()
        self.Running()
        return self.Terminate()


    def Start(self):
        rospy.loginfo("## [flight] Start\n ##")

        # Initialize flag
        self.has_updated_setpoint = False
        self.is_start = True
        self.landing_switching_mod2 = False
        self.landing_switching_mod1 = False

        # Initialize setpoint
        self.prev_global_nearest_sp_idx = 0
        self.prev_local_nearest_sp_idx = 0


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
            self.UpdateSetpoint()
            self.has_updated_setpoint = True
            
            # Check arrived
            if self.IsReached() or self.landing_switching_mod1 == True or self.landing_switching_mod2 == True:
                rospy.loginfo("[flight] break. landing_switching_mod1: %d, landing_switching_mod2: %d\n", self.landing_switching_mod1, self.landing_switching_mod2)
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
        rospy.loginfo_throttle(0.1, "### [flight] IsReached ###")
        if self.setpoint.is_global:
            dist = DistanceFromLatLonInMeter3D(self.setpoints.geopath.poses[-1].pose.position, self.ego_geopose.position)
            rospy.loginfo_throttle(0.1, "[flight] IsReached dist %f / sp: %f, %f, %f / ego: %f, %f, %f\n", dist, 
                            self.setpoints.geopath.poses[-1].pose.position.longitude, self.setpoints.geopath.poses[-1].pose.position.latitude, self.setpoints.geopath.poses[-1].pose.position.altitude, 
                            self.ego_geopose.position.longitude, self.ego_geopose.position.latitude, self.ego_geopose.position.altitude)
        else:
            dist = Distance3D(self.setpoints.pose_array.poses[-1].position, self.ego_pose.position)
            rospy.loginfo_throttle(0.1, "[flight] IsReached dist %f / sp: %f, %f, %f / ego: %f, %f, %f\n", dist, 
                            self.setpoints.pose_array.poses[-1].position.x, self.setpoints.pose_array.poses[-1].position.y, self.setpoints.pose_array.poses[-1].position.z, 
                            self.ego_pose.position.x, self.ego_pose.position.y, self.ego_pose.position.z)

        if dist < self.reached_dist_th_m:
            return True
        else: 
            return False

    def UpdateSetpoint(self):
        rospy.loginfo_throttle(0.1, "### [flight] UpdateSetpoint ###")

        optimal_setpoint = GetOptimalSetpoint(self.setpoints, self.ego_geopose, self.ego_pose, 
                                    self.roi_th_m, self.prev_global_nearest_sp_idx, self.prev_local_nearest_sp_idx)
        geopose = optimal_setpoint[0]
        self.prev_global_nearest_sp_idx = optimal_setpoint[1]
        pose = optimal_setpoint[2]
        self.prev_local_nearest_sp_idx = optimal_setpoint[3]

        rospy.loginfo_throttle(0.1, "[flight] sp length: %d, global updated sp: %d, local updated sp: %d", len(self.setpoints.pose_array.poses),
                                    self.prev_global_nearest_sp_idx, self.prev_local_nearest_sp_idx)

        if self.setpoints.is_global:
            self.setpoint.is_global = True

            self.setpoint.geopose = geopose
            self.setpoint.pose = pose
            rospy.loginfo_throttle(0.1, "[flight] setpoint: global\n")
        else:
            self.setpoint.is_global = False
            self.setpoint.is_setpoint_position = False
            rospy.loginfo_throttle(0.1, "[flight] setpoint: setpoint_raw")


            # transform target pose from camera_link to base_link
            try:
                transform = self.tfBuffer.lookup_transform('base_link', "map", rospy.Time())
                p = PoseStamped()
                p.pose = pose
                transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)

            except:
                rospy.logerr("[flight state] transform err")
                
            self.setpoint.vel.linear.x = XY_Vel(transformed_pose.pose.position.x, 1.0, 1.2)
            self.setpoint.vel.linear.y = XY_Vel(transformed_pose.pose.position.y, 1.0, 1.2)
            self.setpoint.vel.linear.z = Z_Vel(transformed_pose.pose.position.z, 0.9, 1.5)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f", transformed_pose.pose.position.x, self.setpoint.vel.linear.x)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f", transformed_pose.pose.position.y, self.setpoint.vel.linear.y)
            rospy.loginfo_throttle(0.1, "err: %f, vel: %f\n", transformed_pose.pose.position.z, self.setpoint.vel.linear.z)

            target_yaw_rad = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])[2]
            ego_yaw_rad = euler_from_quaternion([self.ego_pose.orientation.x,self.ego_pose.orientation.y,self.ego_pose.orientation.z,self.ego_pose.orientation.w])[2]
            v_yaw = YawRateRad(target_yaw_rad - ego_yaw_rad, 0.7)
            self.setpoint.yaw_rate.orientation.x = v_yaw[0]
            self.setpoint.yaw_rate.orientation.y = v_yaw[1]
            self.setpoint.yaw_rate.orientation.z = v_yaw[2]
            self.setpoint.yaw_rate.orientation.w = v_yaw[3]

            self.setpoint.geopose = geopose
            self.setpoint.pose = pose