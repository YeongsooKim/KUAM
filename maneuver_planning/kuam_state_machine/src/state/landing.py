import rospy
import smach
import state
import copy
from math import pi
from math import tan

import tf2_ros
import tf2_geometry_msgs

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from visualization_msgs.msg import Marker,MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from kuam_msgs.msg import LandingState

class Landing(smach.State, state.Base):
    def __init__(self):
        smach.State.__init__(self, input_keys=['setpoint', 'setpoints'], 
                                output_keys=['setpoint', 'setpoints'], 
                                outcomes=['emerg', 'manual'])
        state.Base.__init__(self)

        self.transition = 'none'
        
        # Flag
        self.landing_state = LandingState()
        self.landing_state.is_detected = False
        self.landing_state.is_land = False
        
        # Target state
        self.target_id = 0 # Change to param
        self.target_pose = Pose()
        
        # Param
        self.landing_threshold_m = None
        self.landing_standby_alt_m = 5.0
        self.virtual_border_angle = [10, 20]

        # tf
        self.tfBuffer = None
        self.listener = None

        self.marker_array = MarkerArray()

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
            if self.landing_state.is_detected == True:
                self.setpoint.pose = self.target_pose

            # Update landing state
            self.UpdateLandingState()

            rate.sleep()

    def Terminate(self, userdata):
        trans = self.transition

        self.transition = 'none'
        return trans

    '''
    Util functions
    '''
    def UpdateLandingState(self):
        if self.landing_threshold_m is None:
            pass
        else:
            h = self.ego_pose.position.z

            if h < self.landing_threshold_m:
                self.landing_state.is_land = True

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

        # Cube
        gt_marker = Marker()
        gt_marker.ns = "ground_truth"
        gt_marker.id = 0
        gt_marker.header.frame_id = "map"
        gt_marker.type = gt_marker.CUBE
        gt_marker.action = gt_marker.ADD

        gt_marker.scale.x = 0.2
        gt_marker.scale.y = 0.2
        gt_marker.scale.z = 0.01
        gt_marker.color = blue
        gt_marker.color.a = 1.0
        gt_marker.pose.orientation.x = 0.0
        gt_marker.pose.orientation.y = 0.0
        gt_marker.pose.orientation.z = 0.0
        gt_marker.pose.orientation.w = 1.0
        gt_marker.pose.position.x = 2.6600
        gt_marker.pose.position.y = -2.2974
        gt_marker.pose.position.z = 0.0
        self.marker_array.markers.append(gt_marker)

        # Cube
        gt_marker = Marker()
        gt_marker.ns = "ground_truth"
        gt_marker.id = 1
        gt_marker.header.frame_id = "map"
        gt_marker.type = gt_marker.CUBE
        gt_marker.action = gt_marker.ADD

        gt_marker.scale.x = 0.06
        gt_marker.scale.y = 0.06
        gt_marker.scale.z = 0.01
        gt_marker.color = blue
        gt_marker.color.a = 1.0
        gt_marker.pose.orientation.x = 0.0
        gt_marker.pose.orientation.y = 0.0
        gt_marker.pose.orientation.z = 0.0
        gt_marker.pose.orientation.w = 1.0
        gt_marker.pose.position.x = 2.8
        gt_marker.pose.position.y = -2.5
        gt_marker.pose.position.z = 0.0
        self.marker_array.markers.append(gt_marker)

        # Line strip
        line_centroid = Point()
        line_centroid.x = pose.position.x
        line_centroid.y = pose.position.y
        line_centroid.z = 0.0

        line_marker = Marker()
        line_marker.ns = "vitual_border_edge"
        line_marker.header.frame_id = "map"
        line_marker.type = line_marker.LINE_STRIP
        line_marker.action = line_marker.ADD

        line_marker.scale.x = 0.1

        line_marker.color = green
        a = Point()
        b = Point()
        c = Point()
        d = Point()
        e = Point()
        a.x = 0.0
        a.y = 0.0
        b.x = 1.0
        b.y = 0.0
        c.x = 1.0
        c.y = 1.0
        d.x = 0.0
        d.y = 1.0
        e.x = 0.0
        e.y = 0.0
        line_marker.points.append(a)
        line_marker.points.append(b)
        line_marker.points.append(c)
        line_marker.points.append(d)
        line_marker.points.append(e)
        
        line_marker.pose.orientation.x = qaut[0]
        line_marker.pose.orientation.y = qaut[1]
        line_marker.pose.orientation.z = qaut[2]
        line_marker.pose.orientation.w = qaut[3]
        line_marker.pose.position = line_centroid
        self.marker_array.markers.append(line_marker)

        return self.marker_array

    def BorderEdgeSize(self, pose):
        h = pose.position.z
        
        edge = 2*h*tan(self.virtual_border_angle[0])
        if edge > 10.0:
            edge = 10.0

        return edge

    # def BoderVertex(pose):
    #     size = self.BorderEdgeSize(pose)
    #     half_size = size/2.0

    #     transform_stamped = TransformStamped()
    #     transform_stamped.header.frame_id = 'map'
    #     transform_stamped.header.stamp = rospy.Time.now()
    #     transform_stamped.child_frame_id = 'base_link'
    #     transform_stamped.transform.rotation = pose.orientation

    #     l_top = pose.position
    #     r_top = 
    #     l_bot = 
    #     r_bot = 

        # is_transformed = False
        # try:
        #     transform_stamped = tfBuffer_.lookup_transform('map', 'base_link', rospy.Time())
        #     is_transformed = True
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        #     pass

        

        # pose_transformed = tf2_geometry_msgs.do_transform_pose(msg_pose_stamped, transform)
        # offb_states_[OffbState.LANDING].target_pose = pose_transformed.pose

    def GetYawRad(self, pose):
        orientation = pose.orientation
        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        yaw_rad = euler[2]
        # yaw_deg = yaw_rad*180.0/pi

        return yaw_rad


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