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

class Landing(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints', 'ego_geopose', 'ego_pose', 'ego_vel'], 
                                output_keys=['setpoint', 'setpoints', 'ego_geopose', 'ego_pose', 'ego_vel'], 
                                outcomes=['disarm', 'emerg', 'manual'])
        state.Base.__init__(self)

        self.transition = 'none'
        
        # Flag
        self.landing_state = LandingState()
        self.landing_state.is_detected = False
        self.landing_state.is_land = False
        self.is_init_z = False
        self.is_pass_landing_standby = False
        
        # Target state
        self.target_id = 0 # Change to param
        self.target_pose = Pose()
        
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

        # tf
        self.tfBuffer = None
        self.listener = None

        self.marker_array = MarkerArray()
        self.vb_angle_deg = [self.virtual_border_angle_0_deg, self.virtual_border_angle_1_deg]

        self.z_traj = []
        self.cnt = 0

        '''
        Change target pose and target id to dictionary when the id is more than two
        '''
        
    def execute(self, userdata):
        self.Start(userdata)
        self.Running()
        return self.Terminate(userdata)

    '''
    State functions
    '''
    def Start(self, userdata):
        # Initialize setpoint
        self.setpoint = copy.deepcopy(userdata.setpoint)
        self.setpoints = copy.deepcopy(userdata.setpoints)
        self.ego_pose = copy.deepcopy(userdata.ego_pose)
        self.ego_geopose = copy.deepcopy(userdata.ego_geopose)
        self.ego_vel = copy.deepcopy(userdata.ego_vel)

        self.setpoint.pose = copy.deepcopy(self.ego_pose)
        self.setpoint.vel.linear.x = 0.0
        self.setpoint.vel.linear.y = 0.0
        self.setpoint.vel.linear.z = -0.3

        geopose = copy.deepcopy(self.setpoints.poses[-1].pose)
        geopose.position.altitude = self.landing_standby_alt_m

        self.landing_standby = geopose.position

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
            self.UpdateVirtualBorder()

            # Update setpoint
            self.UpdateSetpoint()

            # Update landing state
            self.UpdateLandingState()

            rate.sleep()

    def Terminate(self, userdata):
        trans = self.transition

        self.transition = 'none'
        return trans


    '''
    Process functions
    '''
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
        
        yaw_rad = self.GetYawRad(pose)

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
            h = self.ego_geopose.position.altitude

            if h < self.landing_threshold_m:
                self.landing_state.is_land = True

    def UpdateSetpoint(self):
        if self.is_pass_landing_standby == False:
            if self.IsLandingStandby():
                self.is_pass_landing_standby = True

        if self.using_aruco:

            if self.landing_state.is_detected and self.is_pass_landing_standby:

                if self.is_init_z == False:
                    self.is_init_z = True

                    dt = 1.0/self.freq
                    init_z = -self.target_pose.position.z

                    cnt = 1
                    while cnt*dt < self.landing_duration_s:
                        self.z_traj.append(self.Z(cnt*dt, init_z, self.landing_duration_s))
                        cnt += 1

                if self.is_init_z:
                    dt = 1.0/self.freq
                    init_x = self.target_pose.position.x
                    init_y = self.target_pose.position.y
                    duration = self.landing_duration_s - self.cnt*dt

                    x_traj = []
                    y_traj = []

                    cnt = 1
                    while cnt*dt < duration:
                        x_traj.append(self.X(cnt*dt, init_x, duration))
                        y_traj.append(self.Y(cnt*dt, init_y, duration))
                        cnt += 1

                    if len(x_traj) > 1:
                        self.setpoint.vel.linear.x = (x_traj[1] - x_traj[0])/dt
                        self.setpoint.vel.linear.y = (y_traj[1] - y_traj[0])/dt
                    else:
                        self.setpoint.vel.linear.x = 0.0                    
                        self.setpoint.vel.linear.y = 0.0

                    if self.cnt + 1 < len(self.z_traj):
                        self.setpoint.vel.linear.z = (self.z_traj[self.cnt + 1] - self.z_traj[self.cnt])/dt
                        self.cnt += 1
                        # rospy.logerr("x: %f, y: %f, z: %f" %(self.target_pose.position.x, self.target_pose.position.y, self.target_pose.position.z))
        # only gps
        else :
            if self.is_pass_landing_standby:
                self.setpoint.vel.linear.x = 0.0
                self.setpoint.vel.linear.y = 0.0
                self.setpoint.vel.linear.z = -0.3


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
        a = -init_x/(duration**2)
        x = a*(t - duration)**2 + init_x
        return x

    def Y(self, t, init_y, duration):
        a = -init_y/(duration**2)
        y = a*(t - duration)**2 + init_y
        return y

    def Z(self, t, init_z, duration):
        # z = a (x - last_time)**4  -> (0, init_z), (last_time, 0)
        a = init_z/duration**4
        z = a*(t - duration)**4
        return z


    def IsLandingStandby(self):
        dist = DistanceFromLatLonInMeter3D(self.landing_standby, self.ego_geopose.position)
        if dist < self.standby_dist_th_m:
            return True
        else:
            return False

    '''
    Callback functions
    '''
    def MarkerCB(self, msg):
        self.target_id = msg.id

        if msg.is_detected == True:
            msg_pose_stamped = PoseStamped()
            msg_pose_stamped.header = msg.header
            msg_pose_stamped.pose = msg.pose
            # transform from camera_link to base_link
            is_transformed = False
            try:
                transform = self.tfBuffer.lookup_transform('base_link', msg_pose_stamped.header.frame_id, rospy.Time())
                is_transformed = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            if is_transformed == True:
                pose_transformed = tf2_geometry_msgs.do_transform_pose(msg_pose_stamped, transform)
                if self.IsValid(pose_transformed):
                    self.landing_state.is_detected = True
                    self.target_pose = pose_transformed.pose

                else:
                    self.landing_state.is_detected = False