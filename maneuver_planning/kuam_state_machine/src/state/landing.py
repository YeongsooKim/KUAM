import rospy
import smach
import state
import copy
from utils import *
from math import pi
from math import tan
from math import atan2

import tf2_ros
import tf2_geometry_msgs

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from kuam_msgs.msg import LandingState
from geographic_msgs.msg import GeoPoseStamped

class Landing(smach.State, state.Base):
    def __init__(self, ego_geopose, ego_pose, ego_vel, setpoint, setpoints):
        smach.State.__init__(self, input_keys=[], 
                                output_keys=[], 
                                outcomes=['disarm', 'emerg', 'manual'])
        state.Base.__init__(self)
        
        # Flag
        self.landing_state = LandingState()
        self.landing_state.is_detected = False
        self.landing_state.is_land = False
        self.landing_state.is_pass_landing_standby = False
        
        # Param
        self.landing_threshold_m = None
        self.landing_standby_alt_m = 5.0
        self.virtual_border_angle_0_deg = 20.0
        self.virtual_border_angle_1_deg = 10.0
        self.virtual_border_max_side_m = 6.0
        self.alt_division_m = 10.0
        self.standby_dist_th_m = 1.5
        self.landing_duration_s = 80
        self.using_aruco = False
        
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

        self.marker_array = MarkerArray()
        self.vb_angle_deg = [self.virtual_border_angle_0_deg, self.virtual_border_angle_1_deg]

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
        # Initialize flag
        self.landing_state.is_land = False
        self.landing_state.is_detected = False
        self.landing_state.is_pass_landing_standby = False

        # Initialize setpoint
        geopose = self.setpoints.poses[-1].pose
        geopose.position.altitude = self.landing_standby_alt_m
        self.landing_standby = geopose.position
        self.orientation = self.ego_geopose.orientation

        self.GenInitTrajectory(self.ego_pose, self.landing_standby)

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'disarm') or (self.transition == 'emerg') or (self.transition == 'manual'):
                    break
                else:
                    self.transition = 'none'    

            # Update virtual border
            # self.UpdateVirtualBorder()

            # Update setpoint
            self.UpdateSetpoint()

            # Update landing state
            self.UpdateLandingState()

            rate.sleep()

    def Terminate(self):
        trans = self.transition
        self.transition = 'none'
        self.is_start = False
        return trans


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

    def UpdateVirtualBorder(self):
        # clear marker_array
        del self.marker_array.markers[:]

        red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
        blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)
        pose = self.ego_pose

        # Cube
        MARGIN = 1.0 # meter
        cube_centroid = Point()
        cube_centroid.x = pose.position.x
        cube_centroid.y = pose.position.y
        cube_centroid.z = pose.position.z/2
        
        z_scale = pose.position.z + MARGIN
        
        yaw_rad = GetYawRad(pose.orientation)

        qaut = quaternion_from_euler(0.0, 0.0, yaw_rad)
        cube_marker = Marker()
        cube_marker.ns = "virtual_boder_cube"
        cube_marker.header.frame_id = "map"
        cube_marker.type = cube_marker.CUBE
        cube_marker.action = cube_marker.ADD

        cube_marker.scale.x = self.BorderEdgeSize(pose)
        cube_marker.scale.y = self.BorderEdgeSize(pose)
        cube_marker.scale.z = z_scale
        cube_marker.color = red
        cube_marker.color.a = 0.4
        cube_marker.pose.orientation.x = qaut[0]
        cube_marker.pose.orientation.y = qaut[1]
        cube_marker.pose.orientation.z = qaut[2]
        cube_marker.pose.orientation.w = qaut[3]
        cube_marker.pose.position = cube_centroid
        self.marker_array.markers.append(cube_marker)

        return self.marker_array

    def UpdateLandingState(self):
        if self.landing_threshold_m is None:
            pass
        else:
            if self.using_aruco:
                if self.landing_state.is_detected:
                    h = -self.target_pose.position.z

                    if h < self.landing_threshold_m:
                        self.landing_state.is_land = True
            else:
                h = self.ego_geopose.position.altitude
                
                if h < self.landing_threshold_m:
                        self.landing_state.is_land = True

    def UpdateSetpoint(self):
        if self.landing_state.is_pass_landing_standby == False:
            if self.IsLandingStandby():
                self.landing_state.is_pass_landing_standby = True
        
        if self.using_aruco:

            if self.landing_state.is_detected and self.landing_state.is_pass_landing_standby:
                self.setpoint.vel.linear.x = self.XY_Vel(self.target_pose.position.x)
                self.setpoint.vel.linear.y = self.XY_Vel(self.target_pose.position.y)
                self.setpoint.vel.linear.z = self.Z_Vel(self.target_pose.position.z)

                v_yaw = self.YawRate(self.target_pose.orientation)
                self.setpoint.yaw_rate.orientation.x = v_yaw[0]
                self.setpoint.yaw_rate.orientation.y = v_yaw[1]
                self.setpoint.yaw_rate.orientation.z = v_yaw[2]
                self.setpoint.yaw_rate.orientation.w = v_yaw[3]

                self.setpoint.pose.position = self.target_pose.position
                self.setpoint.pose.orientation = self.orientation
            else:
                if self.standby_cnt < len(self.setpoints.poses):
                    self.setpoint.geopose = self.setpoints.poses[self.standby_cnt].pose
                    self.standby_cnt += 1
                else:
                    self.setpoint.geopose.position = self.landing_standby
                    self.setpoint.geopose.orientation = self.orientation

        # only gps
        else :
            if self.landing_state.is_pass_landing_standby:
                self.setpoint.vel.linear.x = 0.0
                self.setpoint.vel.linear.y = 0.0
                self.setpoint.vel.linear.z = -0.3
                self.setpoint.pose.orientation = self.orientation
            else:
                if self.standby_cnt < len(self.setpoints.poses):
                    self.setpoint.geopose = self.setpoints.poses[self.standby_cnt].pose
                    self.standby_cnt += 1
                else:
                    self.setpoint.geopose.position = self.landing_standby
                    self.setpoint.geopose.orientation = self.orientation
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

    def BorderEdgeSize(self, pose):
        h = pose.position.z

        if h > self.alt_division_m:
            angle = Deg2Rad(self.vb_angle_deg[0])
        else: 
            angle = Deg2Rad(self.vb_angle_deg[1])

        edge = 2*h*tan(angle)

        return edge

    def GetYawRad(self, pose):
        orientation = pose.orientation
        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw_rad = euler[2]
        # yaw_deg = yaw_rad*180.0/pi

        return yaw_rad

    def X(self, t, init_x, duration):
        # a = init_x/(duration**2)
        # x = a*(t - duration)**2
        a = -1.5*init_x/duration
        x = a*(t-duration)
        return x

    def Y(self, t, init_y, duration):
        # a = init_y/(duration**2)
        # y = a*(t - duration)**2
        a = -1.5*init_y/duration
        y = a*(t-duration)
        return y

    def Z(self, t, init_z, duration):
        # z = a (x - last_time)**4  -> (0, init_z), (last_time, 0)
        a = init_z/duration**4
        z = a*(t - duration)**4
        return z

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

    def IsLandingStandby(self):
        dist = DistanceFromLatLonInMeter3D(self.landing_standby, self.ego_geopose.position)
        if dist < self.standby_dist_th_m:
            return True
        else:
            return False

    def MapTarget(self, pose, id):
        theta_rad = GetYawRad(pose.orientation)
        big_side = 0.25
        small_side = 0.2

        if id == 0:
            x_marker = -big_side
            y_marker = -big_side
        if id == 1:
            x_marker = +big_side
            y_marker = -big_side
        if id == 2:
            x_marker = +big_side
            y_marker = +big_side
        if id == 3:
            x_marker = -big_side
            y_marker = +big_side
        if id == 4:
            x_marker = -small_side
            y_marker = 0.0
        if id == 5:
            x_marker = 0.0
            y_marker = -small_side
        if id == 6:
            x_marker = +small_side
            y_marker = 0.0
        if id == 7:
            x_marker = 0.0
            y_marker = +small_side
        if id == 8:
            x_marker = 0.0
            y_marker = 0.0

        pose.position.x += self.X_Camera(x_marker, y_marker, theta_rad)
        pose.position.y += self.Y_Camera(x_marker, y_marker, theta_rad)
        return pose

    def X_Camera(self, x_marker, y_marker, theta_rad):
        x_camera = x_marker*cos(theta_rad) + y_marker*sin(theta_rad)
        return x_camera

    def Y_Camera(self, x_marker, y_marker, theta_rad):
        y_camera = x_marker*sin(theta_rad) - y_marker*cos(theta_rad)
        return y_camera

    '''
    Callback functions
    '''
    def MarkerCB(self, msg):
        if not self.is_start:
            return

        target_poses = []
        for ac_state in msg.aruco_states:
            if not ac_state.is_detected:
                continue

            # transform from camera_link to base_link
            try:
                transform = self.tfBuffer.lookup_transform('base_link', ac_state.header.frame_id, rospy.Time())
                p = PoseStamped()
                p.header = ac_state.header
                p.pose = self.MapTarget(ac_state.pose, ac_state.id)
                transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)
                if self.IsValid(transformed_pose):
                    target_poses.append(transformed_pose.pose)

                else:
                    self.landing_state.is_detected = False
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        
        if not len(target_poses) == 0:
            self.landing_state.is_detected = True

            sum_x = 0.0; sum_y = 0.0; sum_z = 0.0
            for pose in target_poses:
                sum_x += pose.position.x
                sum_y += pose.position.y
                sum_z += pose.position.z

            self.target_pose.position.x = sum_x/len(target_poses)
            self.target_pose.position.y = sum_y/len(target_poses)
            self.target_pose.position.z = sum_z/len(target_poses)
            self.target_pose.orientation = target_poses[0].orientation