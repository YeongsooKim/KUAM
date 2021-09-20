#!/usr/bin/env python2
import rospy
import tf2_ros
import tf2_geometry_msgs
import copy
from enum import Enum

# Smach
import smach
from smach_ros import IntrospectionServer
from smach import CBState

# Message
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from uav_msgs.msg import Chat
from kuam_msgs.msg import TransReq
from kuam_msgs.msg import Mode
from kuam_msgs.msg import Task
from kuam_msgs.msg import Setpoint
from kuam_msgs.msg import ArucoStates
from kuam_msgs.msg import LandingState
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

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
    ALTCTL = 3


'''
Parameters. Defined by ros param
'''
freq_ = 1.0 
battery_th_ = 5.0
alt_offset_m_ = 0.0

'''
State Machines
'''
# Create a SMACH state machine
sm_mode_ = smach.StateMachine(outcomes=['finished'], output_keys=[])
sm_offb_ = smach.StateMachine(outcomes=['emerg', 'manual', 'altctl'], input_keys=[])

# Init state variable
ego_geopose_ = GeoPose()
ego_pose_ = Pose()
ego_vel_ = Twist()
setpoint_ = Setpoint()
setpoints_ = GeoPath()

# Init state
offb_states_ = {}
offb_states_[OffbState.STANDBY] = Standby(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.ARM] = Arm(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.TAKEOFF] = Takeoff(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.HOVERING] = Hovering(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.FLIGHT] = Flight(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.LANDING] = Landing(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)

# Init transition
state_transitions_ = []
mode_transitions_ = 'manual'
prev_mode_transitions_ = 'manual'

# Etc
prev_kuam_mode_ = "none"
cur_px4_mode_ = "none"
prev_px4_mode_ = "none"
gps_home_alt_m_ = None
standby_geopose_ = GeoPose()

# Flag
is_init_gps_alt_ = False

'''
Util functions
'''
def GetSMStatus():
    cur_kuam_mode = sm_mode_.get_active_states()[0]
    cur_state = sm_offb_.get_active_states()[0]

    return [cur_kuam_mode, cur_state]


'''
Callback functions
'''
def TaskCB(msg):
    state_transitions_.append(msg.task)

    global setpoints_
    setpoints_.header = msg.geopath.header
    setpoints_.poses = msg.geopath.poses


def ModeCB(msg):
    global mode_transitions_
    global prev_mode_transitions_
    global cur_px4_mode_
    # mode update
    mode_transitions_ = msg.kuam
    cur_px4_mode_ = msg.px4

    # Check state machine mode change
    cur_kuam_mode, cur_state = GetSMStatus()
    if (cur_kuam_mode == 'OFFBOARD') and (cur_state != 'None'):
        if (mode_transitions_ == 'manual') or (mode_transitions_ == 'emerg') or (mode_transitions_ == 'altctl'):
            offb_states_[OffbState[cur_state]].transition = mode_transitions_
            prev_mode_transitions_ = mode_transitions_
        else:
            mode_transitions_ = prev_mode_transitions_

def EgoLocalPoseCB(msg):
    global ego_pose_
    ego_pose_.position = msg.pose.pose.position
    ego_pose_.orientation = msg.pose.pose.orientation

def EgoGlobalPoseCB(msg):
    global gps_home_alt_m_
    global is_init_gps_alt_
    if is_init_gps_alt_ == False:
        is_init_gps_alt_ = True

        gps_home_alt_m_ = msg.altitude

    global standby_geopose_
    standby_geopose_.position.longitude = msg.longitude
    standby_geopose_.position.latitude = msg.latitude
    standby_geopose_.position.altitude = 0.0

    global ego_geopose_
    ego_geopose_.position.longitude = msg.longitude
    ego_geopose_.position.latitude = msg.latitude
    ego_geopose_.position.altitude = ego_pose_.position.z
    ego_geopose_.orientation = ego_pose_.orientation

def EgoVelCB(msg):
    global ego_vel_
    ego_vel_.linear = msg.twist.linear
    ego_vel_.angular = msg.twist.angular

def CommandCB(msg):
    if msg.msg == "local_z":
        alt = msg.point.z
        cur_kuam_mode, cur_state = GetSMStatus()

        if (cur_kuam_mode == "OFFBOARD") and (cur_state == "HOVERING"):
            offb_states_[OffbState[cur_state]].setpoint.geopose.position.altitude = alt

def BatteryCB(msg):
    global mode_transitions_
    global prev_mode_transitions_
    percentage = msg.percentage
    voltage = msg.voltage

    if percentage < battery_th_:
        mode_transitions_ = 'emerg'

        cur_kuam_mode, cur_state = GetSMStatus()
        if (cur_kuam_mode == 'OFFBOARD') and (cur_state != 'None'):
            offb_states_[OffbState[cur_state]].transition = mode_transitions_
            prev_mode_transitions_ = mode_transitions_


'''
Process functions
'''
def DoTransition():
    cur_kuam_mode, cur_state = GetSMStatus()

    if (len(state_transitions_) > 0) and (cur_kuam_mode == 'OFFBOARD') and (cur_state != 'None'):
        trans = state_transitions_.pop(0)
        offb_states_[OffbState[cur_state]].transition = trans

def SetpointPub():
    # Publish setpoint
    cur_kuam_mode, cur_state = GetSMStatus()
    geopose = GeoPose()
    vel = Twist()

    landing_state = LandingState()
    if cur_kuam_mode == "OFFBOARD" and cur_state != "None":
        geopose = copy.deepcopy(setpoint_.geopose)
        vel = copy.deepcopy(setpoint_.vel)
        if cur_state == "LANDING":
            landing_state = offb_states_[OffbState[cur_state]].landing_state
        else:
            landing_state.is_detected = False
            landing_state.is_land = False
    else:
        global standby_geopose_
        geopose.position = copy.deepcopy(standby_geopose_.position)
        landing_state.is_detected = False
        landing_state.is_land = False

    if is_init_gps_alt_ == True:
        msg = Setpoint()
        global gps_home_alt_m_
        if cur_state == "LANDING":
            is_valid = True

            msg.landing_state = landing_state
            if offb_states_[OffbState[cur_state]].landing_state.is_pass_landing_standby and \
                (offb_states_[OffbState[cur_state]].landing_state.is_detected or offb_states_[OffbState[cur_state]].using_aruco == False):

                msg.header.frame_id = "base_link"
                msg.header.stamp = rospy.Time.now()
                msg.vel = vel
                msg.pose = setpoint_.pose
                msg.yaw_rate = setpoint_.yaw_rate
                msg.is_global = False
            else:
                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.now()
                msg.geopose = geopose
                msg.local_height_m = msg.geopose.position.altitude
                msg.home_altitude_m = gps_home_alt_m_
                msg.geopose.position.altitude += (gps_home_alt_m_ - alt_offset_m_)
                msg.is_global = True
        else:
            if geopose.position.latitude == 0 or geopose.position.longitude == 0:
                is_valid = False
            else:
                is_valid = True

                msg.header.frame_id = "map"
                msg.header.stamp = rospy.Time.now()
                msg.geopose = geopose
                msg.local_height_m = msg.geopose.position.altitude
                msg.home_altitude_m = gps_home_alt_m_
                msg.geopose.position.altitude += (gps_home_alt_m_ - alt_offset_m_)
                msg.is_global = True

        if is_valid:
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
            if cur_px4_mode_ != "OFFBOARD" and cur_px4_mode_ != "EMERG" and cur_px4_mode_ != "ALTCTL":
                is_request = True
        elif cur_kuam_mode == "ALTCTL":
            if cur_px4_mode_ != "OFFBOARD" and cur_px4_mode_ != "EMERG" and cur_px4_mode_ != "MANUAL":
                is_request = True
        elif cur_kuam_mode == "OFFBOARD" and cur_state != "None":
            if cur_px4_mode_ != "MANUAL" and cur_px4_mode_ != "ALTCTL" and cur_px4_mode_ != "EMERG":
                is_request = True
        elif cur_kuam_mode == "EMERG":
            if cur_px4_mode_ != "MANUAL" and cur_px4_mode_ != "ALTCTL" and cur_px4_mode_ != "OFFBOARD":
                is_request = True

    if is_request:
        prev_kuam_mode_ = cur_kuam_mode
        prev_px4_mode_ = cur_px4_mode_

        trans_req_msg = TransReq()
        trans_req_msg.mode.kuam = cur_kuam_mode
        trans_req_msg.mode.px4 = cur_px4_mode_
        trans_req_msg.state = cur_state
        trans_req_pub.publish(trans_req_msg)

def ProcessCB(timer):
    DoTransition()
    TransRequest()
    SetpointPub()

'''
Smach callback function
'''
# define state MANUAL
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['altctl', 'offboard', 'finished'])
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
        if (mode_transitions_ == 'altctl') or (mode_transitions_ == 'offboard') or (mode_transitions_ == 'finished'):
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

# define state ALTCTL
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['manual', 'offboard', 'finished'])
def AltitudeCB(userdata):
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
        if (mode_transitions_ == 'manual') or (mode_transitions_ == 'offboard') or (mode_transitions_ == 'finished'):
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
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['manual', 'altctl'])
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
    rospy.init_node('state_machine', disable_signals=False)
    	
    offb_states_[OffbState.LANDING].tfBuffer = tf2_ros.Buffer()
    offb_states_[OffbState.LANDING].listener = tf2_ros.TransformListener(offb_states_[OffbState.LANDING].tfBuffer)

    '''
    Initialize Parameters
    '''
    data_ns = rospy.get_param("~data_ns")
    freq_ = rospy.get_param("~process_freq")
    takeoff_alt_m = rospy.get_param("~takeoff_alt_m")
    reached_dist_th_m = rospy.get_param("~reached_dist_th_m")
    guidance_dist_th_m = rospy.get_param("~guidance_dist_th_m")
    landing_threshold_m = rospy.get_param("~landing_threshold_m")
    landing_standby_alt_m = rospy.get_param("~landing_standby_alt_m")
    standby_dist_th_m = rospy.get_param("~standby_dist_th_m")
    maf_buf_size = rospy.get_param("~maf_buf_size")
    battery_th_ = rospy.get_param("~battery_th_per")
    alt_offset_m_ = rospy.get_param("~alt_offset_m")
    using_aruco = rospy.get_param("~using_aruco")
    big_target_mapping = rospy.get_param("~big_target_mapping")
    medium_target_mapping = rospy.get_param("~medium_target_mapping")
    small_target_mapping = rospy.get_param("~small_target_mapping")

    for state in offb_states_.values():
        try:
            state.freq = freq_
        except:
            cur_kuam_mode, cur_state = GetSMStatus()
            rospy.logerr("Fail init %s freq" %(cur_state))

    offb_states_[OffbState.TAKEOFF].takeoff_alt_m = takeoff_alt_m
    offb_states_[OffbState.TAKEOFF].reached_dist_th_m = reached_dist_th_m
    offb_states_[OffbState.FLIGHT].reached_dist_th_m = reached_dist_th_m
    offb_states_[OffbState.FLIGHT].guidance_dist_th_m = guidance_dist_th_m
    offb_states_[OffbState.LANDING].landing_threshold_m = landing_threshold_m
    offb_states_[OffbState.LANDING].landing_standby_alt_m = landing_standby_alt_m
    offb_states_[OffbState.LANDING].standby_dist_th_m = standby_dist_th_m
    offb_states_[OffbState.LANDING].maf_buf_size = maf_buf_size
    offb_states_[OffbState.LANDING].using_aruco = using_aruco
    offb_states_[OffbState.LANDING].big_target_mapping = big_target_mapping
    offb_states_[OffbState.LANDING].medium_target_mapping = medium_target_mapping
    offb_states_[OffbState.LANDING].small_target_mapping = small_target_mapping

    '''
    Initialize ROS
    '''
    # Init subcriber
    rospy.Subscriber("mission_manager/task", Task, TaskCB)
    rospy.Subscriber("mission_manager/mode", Mode, ModeCB)
    rospy.Subscriber("/mavros/global_position/local", Odometry, EgoLocalPoseCB)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, EgoGlobalPoseCB)
    rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, EgoVelCB)
    rospy.Subscriber(data_ns + "/aruco_tracking/target_states", ArucoStates, offb_states_[OffbState.LANDING].MarkerCB)
    rospy.Subscriber(data_ns + "/chat/command", Chat, CommandCB)
    rospy.Subscriber("/mavros/battery", BatteryState, BatteryCB)

    # Init publisher
    setpoint_pub = rospy.Publisher('~setpoint', Setpoint, queue_size=10)
    trans_req_pub = rospy.Publisher('~trans_request', TransReq, queue_size=10)

    # Init timer
    state_machine_timer = rospy.Timer(rospy.Duration(1/freq_), ProcessCB)
    
    '''
    Initialize and execute State machine
    '''
    # Open the container
    with sm_mode_:
        
        # Add states to the container
        smach.StateMachine.add('MANUAL', CBState(ManualCB),
                                transitions={'altctl':'ALTCTL',
                                             'offboard':'OFFBOARD',
                                             'finished':'finished'})
        smach.StateMachine.add('ALTCTL', CBState(AltitudeCB),
                                transitions={'manual':'MANUAL',
                                             'offboard':'OFFBOARD',
                                             'finished':'finished'})
        smach.StateMachine.add('EMERG', CBState(EmergCB),
                                transitions={'manual':'MANUAL',
                                             'altctl':'ALTCTL'})

        # Open the container
        with sm_offb_:

            # Add states to the container
            smach.StateMachine.add('STANDBY', offb_states_[OffbState.STANDBY],
                                    transitions={'arm':'ARM',
                                                'emerg':'emerg',
                                                'altctl':'altctl',
                                                'manual':'manual'})
            smach.StateMachine.add('ARM', offb_states_[OffbState.ARM],
                                    transitions={'disarm':'STANDBY',
                                                'takeoff':'TAKEOFF',
                                                'emerg':'emerg',
                                                'altctl':'altctl',
                                                'manual':'manual'})
            smach.StateMachine.add('TAKEOFF', offb_states_[OffbState.TAKEOFF], 
                                    transitions={'done':'HOVERING',
                                                'emerg':'emerg',
                                                'altctl':'altctl',
                                                'manual':'manual'})
            smach.StateMachine.add('HOVERING', offb_states_[OffbState.HOVERING], 
                                    transitions={'flight':'FLIGHT',
                                                'landing':'LANDING',
                                                'emerg':'emerg',
                                                'altctl':'altctl',
                                                'manual':'manual'})
            smach.StateMachine.add('FLIGHT', offb_states_[OffbState.FLIGHT], 
                                    transitions={'done':'HOVERING',
                                                'emerg':'emerg',
                                                'altctl':'altctl',
                                                'manual':'manual'})
            smach.StateMachine.add('LANDING', offb_states_[OffbState.LANDING], 
                                    transitions={'disarm':'STANDBY',
                                                'emerg':'emerg',
                                                'altctl':'altctl',
                                                'manual':'manual'})

        smach.StateMachine.add('OFFBOARD', sm_offb_,
                                transitions={'emerg':'EMERG',
                                             'altctl':'ALTCTL',
                                             'manual':'MANUAL'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('sm_viewer', sm_mode_,'/viewer')
    intro_server.start()

    # Execute SMACH plan
    outcome = sm_mode_.execute()

    '''
    ROS spin
    '''
    rospy.spin()
