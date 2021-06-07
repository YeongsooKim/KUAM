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
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import BatteryState
from uav_msgs.msg import Chat
from kuam_msgs.msg import TransReq
from kuam_msgs.msg import Mode
from kuam_msgs.msg import LandingState
from kuam_msgs.msg import Task
from kuam_msgs.msg import Setpoint
from kuam_msgs.msg import MarkerState
from kuam_msgs.msg import LandingState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Point
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
Parameters. Defined by ros param
'''
freq_ = 1.0 
virtual_border_angle_ = [10, 20]
battery_th_ = 5.0


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
prev_mode_transitions_ = 'manual'

# Etc
setpoints_ = None
prev_kuam_mode_ = "none"
cur_px4_mode_ = "none"
prev_px4_mode_ = "none"

'''
Util functions
'''
def GetSMStatus():
    cur_kuam_mode = sm_mode_.get_active_states()[0]
    cur_state = sm_offb_.get_active_states()[0]

    return [cur_kuam_mode, cur_state]

def DoTransition():
    cur_kuam_mode, cur_state = GetSMStatus()

    if (len(state_transitions_) > 0) and (cur_kuam_mode == 'OFFBOARD') and (cur_state != 'None'):
        trans = state_transitions_.pop(0)
        offb_states_[OffbState[cur_state]].setpoints = setpoints_
        offb_states_[OffbState[cur_state]].transition = trans

def SetpointPub():
    # Publish setpoint
    cur_kuam_mode, cur_state = GetSMStatus()
    pose = Pose()
    landing_state = LandingState()
    if cur_kuam_mode == "OFFBOARD" and cur_state != "None":
        pose = offb_states_[OffbState[cur_state]].setpoint.pose
        if cur_state == "LANDING":
            landing_state.is_detected = offb_states_[OffbState[cur_state]].is_detected
            landing_state.is_land = offb_states_[OffbState[cur_state]].is_land
        else:
            landing_state.is_detected = False
            landing_state.is_land = False
    else:
        pose.position.x = 0; pose.position.y = 0; pose.position.z = 0
        landing_state.is_detected = False
        landing_state.is_land = False

    msg = Setpoint()
    msg.pose = pose
    msg.landing_state = landing_state
    
    setpoint_pub.publish(msg)

def TransRequest():
    global prev_kuam_mode_
    global cur_px4_mode_
    global prev_px4_mode_
    cur_kuam_mode, cur_state = GetSMStatus()

    is_kuam_mode_changed = False
    if cur_kuam_mode != prev_kuam_mode_:
        is_kuam_mode_changed = True

    is_px4_mode_changed = False
    if cur_px4_mode_ != prev_px4_mode_:
        is_px4_mode_changed = True

    is_request = False
    if is_kuam_mode_changed:
        is_request = True
    else:
        if cur_kuam_mode == "MANUAL":
            if cur_px4_mode_ != "OFFBOARD" and cur_px4_mode_ != "EMERG":
                is_request = True
        elif cur_kuam_mode == "OFFBOARD" and cur_state != "None":
            if cur_px4_mode_ != "MANUAL" and cur_px4_mode_ != "EMERG":
                is_request = True
        elif cur_kuam_mode == "EMERG":
            if cur_px4_mode_ != "MANUAL" and cur_px4_mode_ != "OFFBOARD":
                is_request = True

    if is_request:
        prev_kuam_mode_ = cur_kuam_mode
        prev_px4_mode_ = cur_px4_mode_

        trans_req_msg = TransReq()
        trans_req_msg.mode.kuam = cur_kuam_mode
        trans_req_msg.mode.px4 = cur_px4_mode_
        trans_req_msg.state = cur_state
        trans_req_pub.publish(trans_req_msg)


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
    gt_marker.id = 0
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
    a = ColorRGBA()
    gt_marker.color = blue
    gt_marker.color.a = 1.0
    gt_marker.pose.orientation.x = 0.0
    gt_marker.pose.orientation.y = 0.0
    gt_marker.pose.orientation.z = 0.0
    gt_marker.pose.orientation.w = 1.0
    gt_marker.pose.position.x = 2.8
    gt_marker.pose.position.y = -2.5
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
    global prev_mode_transitions_
    global cur_px4_mode_
    # mode update
    mode_transitions_ = msg.kuam
    cur_px4_mode_ = msg.px4
    
    # Check state machine mode change
    cur_kuam_mode, cur_state = GetSMStatus()
    if cur_kuam_mode == 'OFFBOARD':
        if (mode_transitions_ == 'manual') or (mode_transitions_ == 'emerg'):
            offb_states_[OffbState[cur_state]].transition = mode_transitions_
            prev_mode_transitions_ = mode_transitions_
        else:
            mode_transitions_ = prev_mode_transitions_

def EgoLocalPoseCB(msg):
    for state in offb_states_.values():
        state.ego_pose = msg.pose

def CommandCB(msg):
    if msg.msg == "local_z":
        alt = msg.point.z
        cur_kuam_mode, cur_state = GetSMStatus()

        if (cur_kuam_mode == "OFFBOARD") and (cur_state == "HOVERING"):
            offb_states_[OffbState[cur_state]].setpoint.pose.position.z = alt

def BatteryCB(msg):
    global mode_transitions_
    global prev_mode_transitions_
    percentage = msg.percentage
    voltage = msg.voltage

    if percentage < battery_th_:
        mode_transitions_ = 'emerg'

        cur_kuam_mode = sm_mode_.get_active_states()[0]
        if cur_kuam_mode == 'OFFBOARD':
            cur_state = sm_offb_.get_active_states()[0]
            offb_states_[OffbState[cur_state]].transition = mode_transitions_
            prev_mode_transitions_ = mode_transitions_

def ProcessCB(timer):
    DoTransition()
    SetpointPub()
    TransRequest()
    
    # if (cur_kuam_mode == "OFFBOARD") and (cur_state != "None"):
    #     VirtualBoderPub(offb_states_[OffbState[cur_state]].ego_pose)


'''
Smach callback function
'''
# define state MANUAL
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['offboard', 'finished'])
def ManualCB(userdata):
    ''' 
    Start
    '''
    global prev_mode_transitions_
    global mode_transitions_

    '''
    Running
    '''
    rate = rospy.Rate(freq_)
    while not rospy.is_shutdown():
        # Break condition
        if (mode_transitions_ == 'offboard') or (mode_transitions_ == 'finished'):
            break
        else:
            mode_transitions_ = prev_mode_transitions_

        # Update request px4 mode
        rate.sleep()

    '''
    Terminate
    '''
    prev_mode_transitions_ = mode_transitions_        
    return mode_transitions_

# define state EMERG
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['manual'])
def EmergCB(userdata):
    '''
    Start
    '''
    global prev_mode_transitions_
    global mode_transitions_

    '''
    Running
    '''
    rate = rospy.Rate(freq_)
    while not rospy.is_shutdown():
        # Break condition
        if mode_transitions_ == 'manual':
            break
        else:
            mode_transitions_ = prev_mode_transitions_

        rate.sleep()

    '''
    Terminate
    '''
    prev_mode_transitions_ = mode_transitions_
    return mode_transitions_
                    
if __name__ == '__main__':
    rospy.init_node('state_machine')
    nd_name = rospy.get_name()
    ns_name = rospy.get_namespace()

    offb_states_[OffbState.LANDING].tfBuffer = tf2_ros.Buffer()
    offb_states_[OffbState.LANDING].listener = tf2_ros.TransformListener(offb_states_[OffbState.LANDING].tfBuffer)

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
    battery_th_ = rospy.get_param(nd_name + "/battery_th_per")

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
    rospy.Subscriber(ns_name + "mission_manager/task", Task, TaskCB)
    rospy.Subscriber(ns_name + "mission_manager/mode", Mode, ModeCB)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, EgoLocalPoseCB)
    rospy.Subscriber(data_ns + "/aruco_tracking/target_state", MarkerState, offb_states_[OffbState.LANDING].MarkerCB)
    rospy.Subscriber(data_ns + "/chat/command", Chat, CommandCB)
    rospy.Subscriber("/mavros/battery", BatteryState, BatteryCB)

    # Init publisher
    setpoint_pub = rospy.Publisher(nd_name + '/setpoint', Setpoint, queue_size=10)
    trans_req_pub = rospy.Publisher(nd_name + '/trans_request', TransReq, queue_size=10)
    marker_pub = rospy.Publisher(nd_name + '/test_marker', MarkerArray, queue_size=10)


    # Init timer
    state_machine_timer = rospy.Timer(rospy.Duration(1/freq_), ProcessCB)
    
    '''
    Initialize and execute State machine
    '''
    # Open the container
    with sm_mode_:
        sm_mode_.userdata.setpoint = Setpoint()
        sm_mode_.userdata.setpoints = PoseArray()

        # Add states to the container
        smach.StateMachine.add('MANUAL', CBState(ManualCB),
                                transitions={'offboard':'OFFBOARD',
                                             'finished':'finished'})
        smach.StateMachine.add('EMERG', CBState(EmergCB),
                                transitions={'manual':'MANUAL'})

        # Open the container
        with sm_offb_:

            # Add states to the container
            smach.StateMachine.add('STANDBY', offb_states_[OffbState.STANDBY], 
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
