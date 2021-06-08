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
from kuam_msgs.msg import AlphaEval

class Landing(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints', 'ego_pose', 'ego_vel'], 
                                output_keys=['setpoint', 'setpoints', 'ego_pose', 'ego_vel'], 
                                outcomes=['emerg', 'manual'])
        state.Base.__init__(self)

        self.transition = 'none'
        
        # Flag
        self.landing_state = LandingState()
        self.landing_state.is_detected = False
        self.landing_state.is_land = False
        self.is_init_z = False
        
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

        # tf
        self.tfBuffer = None
        self.listener = None

        self.marker_array = MarkerArray()
        self.vb_angle_deg = [self.virtual_border_angle_0_deg, self.virtual_border_angle_1_deg]
        self.alpha_eval = AlphaEval()
        self.alpha_list = []

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
        self.setpoints = copy.deepcopy(userdata.setpoints)
        self.ego_pose = copy.deepcopy(userdata.ego_pose)
        self.ego_vel = copy.deepcopy(userdata.ego_vel)
        self.setpoint.pose = self.setpoints.poses[-1]
        self.setpoint.pose.position.z = self.landing_standby_alt_m

    def Running(self):
        rate = rospy.Rate(self.freq)
        while not rospy.is_shutdown():
            # Break condition
            if self.transition != 'none':
                if (self.transition == 'emerg') or (self.transition == 'manual'):
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
        userdata.ego_pose = copy.deepcopy(self.ego_pose)
        userdata.ego_vel = copy.deepcopy(self.ego_vel)

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
            h = self.ego_pose.position.z

            if h < self.landing_threshold_m:
                self.landing_state.is_land = True

    def UpdateSetpoint(self):
        if self.landing_state.is_detected == True:
            if self.is_init_z == False:
                self.is_init_z = True
                # self.init_vel_z_ms = self.ego_vel.linear.z

                init_z = self.ego_pose.position.z
                dt = 1.0/self.freq

                cnt = 1
                while cnt*dt < 80:
                    self.z_traj.append(self.Z(cnt*dt, init_z))
                    cnt += 1

            if self.is_init_z:
                dt = 1.0/self.freq
                if self.cnt + 1 < len(self.z_traj):
                    self.setpoint.pose = self.target_pose
                    self.setpoint.pose.position.z = self.z_traj[self.cnt]
                    self.setpoint.vel.linear.z = (self.z_traj[self.cnt + 1] - self.z_traj[self.cnt])/dt
                    self.cnt += 1
                else:
                    self.setpoint.pose = self.target_pose


            # # Vertical distance to marker
            # h_m = self.ego_pose.position.z

            # # Horizontal distance to marker
            # ego_xy_pos = Point()
            # ego_xy_pos.x = self.ego_pose.position.x
            # ego_xy_pos.y = self.ego_pose.position.y

            # target_xy_pos = Point()
            # target_xy_pos.x = self.target_pose.position.x
            # target_xy_pos.y = self.target_pose.position.y

            # xy_m = Distance2D(ego_xy_pos, target_xy_pos)

            # # Angle between edge of drone to marker and drone to marker horizontal 
            # alpha_deg = Rad2Deg(atan2(h_m, xy_m))


            # # Update setpoint
            # if self.is_init_z:
            #     self.alpha_list.append(alpha_deg)
            #     z_vel = min(self.alpha_list)*0.001
            #     if z_vel < 0.01:
            #         z_vel = 0
            #     rospy.logwarn(str(z_vel))

            # else:
            #     z_vel = 0.0
            #     rospy.logerr(str(z_vel))



            # self.setpoint.pose = self.target_pose
            # self.setpoint.vel.linear.z = z_vel

            # # Publish evaluate alpha
            # self.alpha_eval.alpha_deg.data = alpha_deg
            # self.alpha_eval.d_m.data = xy_m
            # self.alpha_eval.h_m.data = h_m
            # self.alpha_eval.target_pos = self.target_pose.position

            
            # self.setpoint.pose = self.target_pose

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

    def Z(self, t, init_z):
        # z = a (x - last_time)**4  -> (0, init_z), (last_time, 0)
        last_time = 100
        a = init_z/last_time**4
        z = a*(t-last_time)**4
        return z



    '''
    Callback functions
    '''
    def MarkerCB(self, msg):
        self.target_id = msg.id

        if msg.is_detected == True:
            msg_pose_stamped = PoseStamped()
            msg_pose_stamped.header = msg.header
            msg_pose_stamped.pose = msg.pose
            # transform from camera_link to map
            is_transformed = False
            try:
                transform = self.tfBuffer.lookup_transform('map', msg_pose_stamped.header.frame_id, rospy.Time())
                is_transformed = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            if is_transformed == True:
                pose_transformed = tf2_geometry_msgs.do_transform_pose(msg_pose_stamped, transform)
                if self.IsValid(pose_transformed):
                    self.landing_state.is_detected = True
                    self.target_pose = pose_transformed.pose
                    # target_pose_pub.publish(pose_transformed)
                else:
                    self.landing_state.is_detected = False

    # # Get virtual border side
    # def GetVBside(self):
    #     '''
    #     ego orientation -> roll, pitch, yaw 
    #     get VBS (virtual border side).
    #     body -> global (trans, yaw)
    #     '''