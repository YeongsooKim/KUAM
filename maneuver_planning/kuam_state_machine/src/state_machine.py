#!/usr/bin/env python

import rospy
import tf2_ros
import tf2_geometry_msgs
from enum import Enum
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
from math import tan

# Smach
import smach
from smach_ros import IntrospectionServer
from smach import CBState

# Message
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA
from kuam_msgs.msg import Task
from kuam_msgs.msg import Setpoint
from kuam_msgs.msg import MarkerState
from kuam_msgs.msg import LandingState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPose
from mavros_msgs.msg import State
from visualization_msgs.msg import Marker,MarkerArray

# State
from state.standby import *
from state.arm import *
from state.docking import *
from state.undocking import *
from state.takeoff import *
from state.hovering import *
from state.flight import *
from state.transition import *
from state.landing import *


'''
Enum classes
'''
# Init state machine state
class OffbState(Enum):
    STANDBY = 0
    ARM = 1
    DOCKING = 2
    UNDOCKING = 3
    TAKEOFF = 4
    HOVERING = 5
    FLIGHT = 6
    TRANSITION = 7
    LANDING = 8

# Init state machine mode
class SMMode(Enum):
    MANUAL = 0
    OFFBOARD = 1
    EMERG = 2


'''
Parameters
'''
freq_ = 1.0
virtual_border_angle_ = [10, 20]


'''
State Machines
'''
# Create a SMACH state machine
sm_mode_ = smach.StateMachine(outcomes=['finished'], output_keys=['setpoint', 'setpoints'])
sm_offb_ = smach.StateMachine(outcomes=['emerg', 'manual'], input_keys=['setpoint', 'setpoints'])

# Init state
offb_states_ = {}
offb_states_[OffbState.STANDBY] = Standby()
offb_states_[OffbState.ARM] = Arm()
offb_states_[OffbState.TAKEOFF] = Takeoff()
offb_states_[OffbState.HOVERING] = Hovering()
offb_states_[OffbState.FLIGHT] = Flight()
offb_states_[OffbState.LANDING] = Landing()

# Init transition
state_transitions_ = []
mode_transitions_ = 'manual'

#
setpoints_ = None
tfBuffer_ = None
listener_ = None


'''
Util functions
'''
def DoTransition():
    cur_state = sm_offb_.get_active_states()[0]

    if (len(state_transitions_) > 0) and (cur_state != 'None'):
        trans = state_transitions_.pop(0)
        offb_states_[OffbState[cur_state]].setpoints = setpoints_
        offb_states_[OffbState[cur_state]].transition = trans

def GetSMStatus():
    cur_mode = sm_mode_.get_active_states()[0]
    cur_state = sm_offb_.get_active_states()[0]

    return [cur_mode, cur_state]

def VirtualBoderPub(pose):
    marker_array = MarkerArray()
    red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)

    # Cube
    MARGIN = 1.0 # meter
    cube_centroid = Point()
    cube_centroid.x = pose.position.x
    cube_centroid.y = pose.position.y
    cube_centroid.z = pose.position.z/2
    
    z_scale = pose.position.z + MARGIN
    
    yaw_rad = GetYawRad(pose)

    qaut = quaternion_from_euler(0.0, 0.0, yaw_rad)
    cube_marker = Marker()
    cube_marker.ns = "virtual_boder_cube"
    cube_marker.header.frame_id = "map"
    cube_marker.type = cube_marker.CUBE
    cube_marker.action = cube_marker.ADD

    cube_marker.scale.x = BorderEdgeSize(pose)
    cube_marker.scale.y = BorderEdgeSize(pose)
    cube_marker.scale.z = z_scale
    a = ColorRGBA()
    cube_marker.color = red
    cube_marker.color.a = 0.4
    cube_marker.pose.orientation.x = qaut[0]
    cube_marker.pose.orientation.y = qaut[1]
    cube_marker.pose.orientation.z = qaut[2]
    cube_marker.pose.orientation.w = qaut[3]
    cube_marker.pose.position = cube_centroid
    marker_array.markers.append(cube_marker)

    # Cube
    gt_marker = Marker()
    gt_marker.ns = "ground_truth"
    gt_marker.header.frame_id = "map"
    gt_marker.type = gt_marker.CUBE
    gt_marker.action = gt_marker.ADD

    gt_marker.scale.x = 0.2
    gt_marker.scale.y = 0.2
    gt_marker.scale.z = 0.01
    a = ColorRGBA()
    gt_marker.color = blue
    gt_marker.color.a = 1.0
    gt_marker.pose.orientation.x = 0.0
    gt_marker.pose.orientation.y = 0.0
    gt_marker.pose.orientation.z = 0.0
    gt_marker.pose.orientation.w = 1.0
    gt_marker.pose.position.x = 2.6600
    gt_marker.pose.position.y = -2.2974
    gt_marker.pose.position.z = 0.0
    marker_array.markers.append(gt_marker)

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
    marker_array.markers.append(line_marker)
    marker_pub.publish(marker_array)

def BorderEdgeSize(pose):
    h = pose.position.z
    
    edge = 2*h*tan(virtual_border_angle_[0])
    if edge > 10.0:
        edge = 10.0

    return edge

# def BoderVertex(pose):
#     size = BorderEdgeSize(pose)
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




def GetYawRad(pose):
    orientation = pose.orientation
    euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    yaw_rad = euler[2]
    # yaw_deg = yaw_rad*180.0/pi

    return yaw_rad

'''
Callback functions
'''
def TaskCB(msg):
    state_transitions_.append(msg.task)
    global setpoints_
    setpoints_ = msg.pose_array

def ModeCB(msg):
    global mode_transitions_
    mode_transitions_ = msg.data

    cur_mode = sm_mode_.get_active_states()[0]
    if cur_mode == 'OFFBOARD':
        cur_state = sm_offb_.get_active_states()[0]
        offb_states_[OffbState[cur_state]].transition = mode_transitions_

def EgoLocalPoseCB(msg):
    for state in offb_states_.values():
        state.ego_pose = msg.pose

def MarkerCB(msg):
    offb_states_[OffbState.LANDING].is_detected = msg.is_detected
    offb_states_[OffbState.LANDING].target_id = msg.id

    if msg.is_detected == True:
        msg_pose_stamped = PoseStamped()
        msg_pose_stamped.header = msg.header
        msg_pose_stamped.pose = msg.pose
        # transform from camera_link to map
        is_transformed = False
        try:
            transform = tfBuffer_.lookup_transform('map', msg_pose_stamped.header.frame_id, rospy.Time())
            is_transformed = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        if is_transformed == True:
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg_pose_stamped, transform)
            offb_states_[OffbState.LANDING].target_pose = pose_transformed.pose

def MavrosStateCB(msg):
    pass

def ProcessCB(timer):
    # Do transition
    DoTransition()

    # Publish
    cur_mode, cur_state = GetSMStatus()
    mode_pub.publish(cur_mode)
    state_pub.publish(cur_state)

    if (cur_state != 'None'):
        VirtualBoderPub(offb_states_[OffbState[cur_state]].ego_pose)

        setpoint_pub.publish(offb_states_[OffbState[cur_state]].setpoint)
        setpoints_pub.publish(offb_states_[OffbState[cur_state]].setpoints)


'''
Smach callback function
'''
# define state MANUAL
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['offboard', 'finished'])
def ManualCB(userdata):
    rate = rospy.Rate(freq_)
    while mode_transitions_ == 'manual':
        rate.sleep()

    if mode_transitions_ == 'offboard':
        return 'offboard'
    elif mode_transitions_ == 'finished':
        return 'finished'

# define state EMERG
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['emerg'])
def EmergCB(userdata):
    rate = rospy.Rate(freq_)
    while mode_transitions_ == 'emerg':
        rate.sleep()

    return 'emerg'


if __name__ == '__main__':
    rospy.init_node('state_machine')
    nd_name = rospy.get_name()
    ns_name = rospy.get_namespace()

    '''
    Initialize Parameters
    '''
    freq_ = rospy.get_param(nd_name + "/freq")
    takeoff_alt_m = rospy.get_param(nd_name + "/takeoff_alt_m")
    dist_thresh_m = rospy.get_param(nd_name + "/reached_dist_th_m")
    guidance_dist_th_m = rospy.get_param(nd_name + "/guidance_dist_th_m")
    data_ns = rospy.get_param(nd_name + "/data_ns")
    target_marker_id = rospy.get_param(nd_name + "/target_marker_id")
    landing_threshold_m = rospy.get_param(nd_name + "/landing_threshold_m")
    landing_standby_alt_m = rospy.get_param(nd_name + "/landing_standby_alt_m")

    for state in offb_states_.values():
        state.freq = freq_

    offb_states_[OffbState.TAKEOFF].takeoff_alt_m = takeoff_alt_m
    offb_states_[OffbState.TAKEOFF].dist_thresh_m = dist_thresh_m
    offb_states_[OffbState.FLIGHT].dist_thresh_m = dist_thresh_m
    offb_states_[OffbState.FLIGHT].guidance_dist_th_m = guidance_dist_th_m
    offb_states_[OffbState.LANDING].landing_threshold_m = landing_threshold_m
    offb_states_[OffbState.LANDING].landing_standby_alt_m = landing_standby_alt_m

    '''
    Initialize ROS
    '''
    # Init subcriber
    rospy.Subscriber(ns_name + "/mission_manager/task", Task, TaskCB)
    rospy.Subscriber(ns_name + "/mission_manager/mode", String, ModeCB)
    rospy.Subscriber("/mavros/state", State, MavrosStateCB)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, EgoLocalPoseCB)
    rospy.Subscriber(data_ns + "/aruco_tracking/target_state", MarkerState, MarkerCB)

    # Init publisher
    mode_pub = rospy.Publisher(nd_name + '/mode', String, queue_size=10)
    state_pub = rospy.Publisher(nd_name + '/state', String, queue_size=10)
    setpoint_pub = rospy.Publisher(nd_name + '/setpoint', Setpoint, queue_size=10)
    setpoints_pub = rospy.Publisher(nd_name + '/setpoints', PoseArray, queue_size=10)
    marker_pub = rospy.Publisher(nd_name + '/test_marker', MarkerArray, queue_size=10)
    landing_state_pub = rospy.Publisher(nd_name + '/landing_state', LandingState, queue_size=10)
    offb_states_[OffbState.LANDING].landing_state_pub = landing_state_pub

    # Init timer
    state_machine_timer = rospy.Timer(rospy.Duration(1/freq_), ProcessCB)

    tfBuffer_ = tf2_ros.Buffer()
    listener_ = tf2_ros.TransformListener(tfBuffer_)

    '''
    Initialize and execute State machine
    '''
    # Open the container
    with sm_mode_:
        sm_mode_.userdata.setpoint = Setpoint()
        sm_mode_.userdata.setpoints = PoseArray()
        sm_mode_.userdata.geo_setpoint = GeoPose()

        # Add states to the container
        smach.StateMachine.add('MANUAL', CBState(ManualCB),
                                transitions={'offboard':'OFFBOARD',
                                             'finished':'finished'})
        smach.StateMachine.add('EMERG', CBState(EmergCB),
                                transitions={'emerg':'EMERG'})

        # Open the container
        with sm_offb_:
            # Add states to the container
            smach.StateMachine.add('STANDBY', offb_states_[OffbState.STANDBY], 
                                    # transitions={'arm':'ARM'})
                                    transitions={'arm':'ARM',
                                                'emerg':'emerg',
                                                'manual':'manual'})
            smach.StateMachine.add('ARM', offb_states_[OffbState.ARM], 
                                    transitions={'disarm':'STANDBY',
                                                'takeoff':'TAKEOFF',
                                                'emerg':'emerg',
                                                'manual':'manual'})
            smach.StateMachine.add('TAKEOFF', offb_states_[OffbState.TAKEOFF], 
                                    transitions={'done':'HOVERING',
                                                'emerg':'emerg',
                                                'manual':'manual'})
            smach.StateMachine.add('HOVERING', offb_states_[OffbState.HOVERING], 
                                    transitions={'flight':'FLIGHT',
                                                'landing':'LANDING',
                                                'emerg':'emerg',
                                                'manual':'manual'})
            smach.StateMachine.add('FLIGHT', offb_states_[OffbState.FLIGHT], 
                                    transitions={'done':'HOVERING',
                                                'emerg':'emerg',
                                                'manual':'manual'})
            smach.StateMachine.add('LANDING', offb_states_[OffbState.LANDING], 
                                    transitions={'emerg':'emerg',
                                                'manual':'manual'})

        smach.StateMachine.add('OFFBOARD', sm_offb_,
                                transitions={'emerg':'EMERG',
                                             'manual':'MANUAL'})

        # smach.StateMachine.add('TRANSITION', CBState(TransitionCB), 
        #                         transitions={'flight':'FLIGHT'})
        # smach.StateMachine.add('DISARM', CBState(DisarmCB), 
        #                         transitions={'standby':'STANDBY'})
        # smach.StateMachine.add('DOCKING', CBState(DockingCB), 
        #                         transitions={'standby':'STANDBY'})
        # smach.StateMachine.add('UNDOCKING', CBState(UndockingCB), 
        #                         transitions={'standby':'STANDBY'})
    

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('sm_viewer', sm_mode_,'/viewer')
    intro_server.start()

    # Execute SMACH plan
    outcome = sm_mode_.execute()

    '''
    ROS spin
    '''
    rospy.spin()
