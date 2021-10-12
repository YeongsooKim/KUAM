#!/usr/bin/env python3

import rospy
import tf2_ros
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
from uav_msgs.msg import PayloadCmd
from kuam_msgs.msg import ArucoStates
from kuam_msgs.msg import Completion
from kuam_msgs.msg import Setpoint
from kuam_msgs.msg import Status
from kuam_msgs.msg import Task
from kuam_msgs.msg import VehicleState
from kuam_msgs.srv import LandRequest
from kuam_msgs.srv import EmergyRequest
from geographic_msgs.msg import GeoPose
from geographic_msgs.msg import GeoPath
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist

# State
from state.standby import *
from state.takeoff import *
from state.hovering import *
from state.flight import *
from state.landing import *


'''
Enum classes
'''
# Init state machine state
class OffbState(Enum):
    STANDBY = 0
    TAKEOFF = 1
    HOVERING = 2
    FLIGHT = 3
    LANDING = 4

# Init state machine mode
class SMMode(Enum):
    MANUAL = 0
    OFFBOARD = 1
    EMERGY = 2
    ALTCTL = 3
    LAND = 4


'''
State Machines
'''
# Create a SMACH state machine
sm_mode_ = smach.StateMachine(outcomes=['finished'], output_keys=[])
sm_offb_ = smach.StateMachine(outcomes=['MANUAL', 'ALTCTL', 'AUTO.RTL', 'AUTO.LAND'], input_keys=[])

# Init state variable
ego_geopose_ = GeoPose()
ego_pose_ = Pose()
ego_vel_ = Twist()
setpoint_ = Setpoint()
setpoints_ = GeoPath()

# Init state
offb_states_ = {}
offb_states_[OffbState.STANDBY] = Standby(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.TAKEOFF] = Takeoff(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.HOVERING] = Hovering(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.FLIGHT] = Flight(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)
offb_states_[OffbState.LANDING] = Landing(ego_geopose_, ego_pose_, ego_vel_, setpoint_, setpoints_)

# Init transition
offb_transition_list_ = []
mode_transitions_ = 'MANUAL'
prev_mode_transitions_ = 'MANUAL'

# Etc
gps_home_alt_m_ = None
standby_geopose_ = GeoPose()

# Flag
is_init_gps_alt_ = False


'''
Process functions
'''
def OffbTransition():
    cur_mode, cur_offb_state = GetSMStatus()

    if (len(offb_transition_list_) > 0) and (cur_mode == 'OFFBOARD') and (cur_offb_state != 'None'):
        trans = offb_transition_list_.pop(0)
        offb_states_[OffbState[cur_offb_state]].transition = trans

def SetpointPub():
    # Publish setpoint
    global is_init_gps_alt_
    if not is_init_gps_alt_:
        return

    cur_mode, cur_offb_state = GetSMStatus()
    # Case of offboard
    if cur_mode == "OFFBOARD" and cur_offb_state != "None":
        if cur_offb_state == "LANDING":
            # using gps only
            if not offb_states_[OffbState[cur_offb_state]].using_camera:
                msg = GetSetpointMsg(copy.deepcopy(setpoint_))
            
            # using gps and camera
            else:
                msg = GetSetpointMsg(copy.deepcopy(setpoint_))
        else:
            msg = GetSetpointMsg(copy.deepcopy(setpoint_))

    #Case of not offboard
    else:
        msg = GetStandbyMsg(standby_geopose_)

    if msg.geopose.position.latitude != 0 and msg.geopose.position.longitude != 0:
        setpoint_pub.publish(msg)

def StatusPub():
    cur_mode, cur_offb_state = GetSMStatus()
    msg = Status()
    msg.mode = cur_mode
    msg.offb_state = cur_offb_state
    status_pub.publish(msg)

def CompletionPub(msg):
    completion_pub.publish(msg)

def ProcessCB(timer):
    OffbTransition()
    SetpointPub()
    StatusPub()


'''
Callback functions
'''
def TaskCB(msg):
    offb_transition_list_.append(msg.task)

    global setpoints_
    setpoints_.header = msg.geopath.header
    setpoints_.poses = msg.geopath.poses

def PayloadCmdCB(msg):
    global mode_transitions_
    global prev_mode_transitions_
    # mode update
    mode_transitions_ = msg.mode

    # Check state machine mode change
    cur_mode, cur_offb_state = GetSMStatus()
    if (cur_mode == 'OFFBOARD') and (cur_offb_state != 'None'):
        if (mode_transitions_ == 'MANUAL') or (mode_transitions_ == 'AUTO.RTL') \
            or (mode_transitions_ == 'ALTCTL') or (mode_transitions_ == 'AUTO.LAND'):
            offb_states_[OffbState[cur_offb_state]].transition = mode_transitions_
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
    standby_geopose_.orientation = ego_pose_.orientation

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
        cur_mode, cur_offb_state = GetSMStatus()

        if (cur_mode == "OFFBOARD") and (cur_offb_state == "HOVERING"):
            offb_states_[OffbState[cur_offb_state]].setpoint.geopose.position.altitude = alt

def BatteryCB(msg):
    global mode_transitions_
    global prev_mode_transitions_
    percentage = msg.percentage
    voltage = msg.voltage

    if percentage < battery_th:
        mode_transitions_ = 'AUTO.RTL'

        cur_mode, cur_offb_state = GetSMStatus()
        if (cur_mode == 'OFFBOARD') and (cur_offb_state != 'None'):
            offb_states_[OffbState[cur_offb_state]].transition = mode_transitions_
            prev_mode_transitions_ = mode_transitions_


'''
Util functions
'''
def GetSMStatus():
    cur_mode = sm_mode_.get_active_states()[0]
    cur_offb_state = sm_offb_.get_active_states()[0]

    return [cur_mode, cur_offb_state]

def GetStandbyMsg(geopose):
    global gps_home_alt_m_

    msg = Setpoint()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    msg.geopose = copy.deepcopy(geopose)
    msg.local_height_m = msg.geopose.position.altitude
    msg.home_altitude_m = gps_home_alt_m_
    msg.geopose.position.altitude += (gps_home_alt_m_ - alt_offset_m)
    msg.is_global = True

    return msg

def GetSetpointMsg(setpoint):
    global gps_home_alt_m_

    msg = Setpoint()
    msg.header.stamp = rospy.Time.now()

    msg.local_height_m = setpoint.geopose.position.altitude
    msg.home_altitude_m = gps_home_alt_m_
    
    msg.geopose = setpoint.geopose
    msg.geopose.position.altitude += (gps_home_alt_m_ - alt_offset_m)
    
    msg.vel = setpoint.vel
    msg.yaw_rate = setpoint.yaw_rate
    
    msg.landing_state = setpoint.landing_state

    msg.target_pose = setpoint.target_pose

    if setpoint.is_global:
        msg.header.frame_id = "map"
        msg.is_global = True
    else:
        msg.header.frame_id = "base_link"
        msg.is_global = False

    return msg


'''
Smach callback function
'''
def ModeProcess(outcomes, func=None):
    ''' 
    Start
    '''
    global prev_mode_transitions_
    global mode_transitions_

    '''
    Running
    '''
    rate = rospy.Rate(freq)
    do_trans = False
    while not rospy.is_shutdown() and not do_trans:
        # Break condition
        if mode_transitions_ != prev_mode_transitions_:
            for outcome in outcomes:
                if mode_transitions_ == outcome:
                    do_trans = True
                    break
            
            if not do_trans:
                mode_transitions_ = prev_mode_transitions_
        
        if func is not None:
            is_complete, output_mode = func()
            if is_complete:
                mode_transitions_ = output_mode
                break

        rate.sleep()

    '''
    Terminate
    '''
    mode = sm_mode_.get_active_states()[0]
    prev_mode_transitions_ = mode_transitions_ 

    return mode_transitions_


# define state MANUAL
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['MANUAL', 'ALTCTL', 'OFFBOARD', 'AUTO.RTL', 'finished'])
def ManualCB(userdata):
    outcome = ModeProcess(['MANUAL', 'ALTCTL', 'OFFBOARD', 'AUTO.RTL', 'finished'])
    return outcome

# define state ALTCTL
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['MANUAL', 'ALTCTL',  'OFFBOARD', 'AUTO.RTL', 'finished'])
def AltitudeCB(userdata):
    outcome = ModeProcess(['MANUAL', 'ALTCTL',  'OFFBOARD', 'AUTO.RTL', 'finished'])
    return outcome
    
# define state LAND
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['ALTCTL', 'MANUAL', 'AUTO.RTL', 'OFFBOARD', 'finished'])
def LandCB(userdata):
    rospy.logwarn("[state_machine] Enter LandCB")
    def LandReuest():
        rospy.logwarn("[state_machine] Service requesting land")
        rospy.wait_for_service('payload_cmd/land_request')
        try:
            land_request = rospy.ServiceProxy('payload_cmd/land_request', LandRequest)
            res = land_request(True)
            return res.is_complete, 'ALTCTL'
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    outcome = ModeProcess(['ALTCTL', 'MANUAL', 'AUTO.RTL', 'OFFBOARD', 'finished'], LandReuest)

    msg = Completion()
    msg.task = "landing"
    msg.is_complete = True
    msg.geopose = ego_geopose_
    CompletionPub(msg)

    rospy.logwarn("End LandCB")
    return outcome
    
# define state EMERGY
@smach.cb_interface(input_keys=[], output_keys=[], outcomes=['MANUAL', 'ALTCTL', 'finished'])
def EmergyCB(userdata):
    rospy.logwarn("Enter EmgeryCB")
    def EmergyReq():
        rospy.logwarn("Service requesting emergy")
        rospy.wait_for_service('payload_cmd/emergy_request')
        try:
            emergy_request = rospy.ServiceProxy('payload_cmd/emergy_request', EmergyRequest)
            res = emergy_request(True)
            return res.is_complete
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

    outcome = ModeProcess(['MANUAL', 'ALTCTL', 'finished'])
    # outcome = ModeProcess(['MANUAL', 'ALTCTL', 'finished'], EmergyReq)
    return outcome
    

if __name__ == '__main__':
    rospy.init_node('state_machine', disable_signals=False)
    	
    offb_states_[OffbState.LANDING].tfBuffer = tf2_ros.Buffer()
    offb_states_[OffbState.LANDING].listener = tf2_ros.TransformListener(offb_states_[OffbState.LANDING].tfBuffer)

    '''
    Initialize Parameters
    '''
    data_ns = rospy.get_param("~data_ns")
    freq = rospy.get_param("~process_freq")
    takeoff_alt_m = rospy.get_param("~takeoff_alt_m")
    reached_dist_th_m = rospy.get_param("~reached_dist_th_m")
    guidance_dist_th_m = rospy.get_param("~guidance_dist_th_m")
    landing_threshold_m = rospy.get_param("~landing_threshold_m")
    battery_th = rospy.get_param("~battery_th_per")
    alt_offset_m = rospy.get_param("~alt_offset_m")
    using_camera = rospy.get_param("~using_camera")

    for state in offb_states_.values():
        try:
            state.freq = freq
        except:
            cur_mode, cur_offb_state = GetSMStatus()
            rospy.logerr("Fail init %s freq" %(cur_offb_state))

    offb_states_[OffbState.TAKEOFF].takeoff_alt_m = takeoff_alt_m
    offb_states_[OffbState.TAKEOFF].reached_dist_th_m = reached_dist_th_m
    offb_states_[OffbState.FLIGHT].reached_dist_th_m = reached_dist_th_m
    offb_states_[OffbState.FLIGHT].guidance_dist_th_m = guidance_dist_th_m
    offb_states_[OffbState.LANDING].reached_dist_th_m = reached_dist_th_m
    offb_states_[OffbState.LANDING].guidance_dist_th_m = guidance_dist_th_m
    offb_states_[OffbState.LANDING].landing_threshold_m = landing_threshold_m
    offb_states_[OffbState.LANDING].using_camera = using_camera

    '''
    Initialize ROS
    '''
    # Init subcriber
    rospy.Subscriber("mission_manager/task", Task, TaskCB)
    rospy.Subscriber("payload_cmd/payload_cmd", PayloadCmd, PayloadCmdCB)
    rospy.Subscriber("/mavros/global_position/local", Odometry, EgoLocalPoseCB)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, EgoGlobalPoseCB)
    rospy.Subscriber("/mavros/local_position/velocity_body", TwistStamped, EgoVelCB)
    rospy.Subscriber(data_ns + "/aruco_tracking/target_states", ArucoStates, offb_states_[OffbState.LANDING].MarkerCB)
    rospy.Subscriber(data_ns + "/vehicle_position/vehicle_state", VehicleState, offb_states_[OffbState.LANDING].VehicleCB)
    rospy.Subscriber(data_ns + "/chat/command", Chat, CommandCB)
    rospy.Subscriber("/mavros/battery", BatteryState, BatteryCB)

    # Init publisher
    setpoint_pub = rospy.Publisher('~setpoint', Setpoint, queue_size=10)
    completion_pub = rospy.Publisher('~completion', Completion, queue_size=10)
    status_pub = rospy.Publisher('~status', Status, queue_size=10)

    offb_states_[OffbState.TAKEOFF].CompletionPub = CompletionPub
    offb_states_[OffbState.FLIGHT].CompletionPub = CompletionPub

    # Init timer
    state_machine_timer = rospy.Timer(rospy.Duration(1/freq), ProcessCB)
    
    '''
    Initialize and execute State machine
    '''
    # Open the container
    with sm_mode_:
        
        # Add states to the container
        smach.StateMachine.add('MANUAL', CBState(ManualCB),
                                transitions={'MANUAL':'MANUAL',
                                             'ALTCTL':'ALTCTL',
                                             'OFFBOARD':'OFFBOARD',
                                             'AUTO.RTL':'EMERGY',
                                             'finished':'finished'})
        smach.StateMachine.add('ALTCTL', CBState(AltitudeCB),
                                transitions={'ALTCTL':'ALTCTL',
                                             'MANUAL':'MANUAL',
                                             'OFFBOARD':'OFFBOARD',
                                             'AUTO.RTL':'EMERGY',
                                             'finished':'finished'})
        smach.StateMachine.add('LAND', CBState(LandCB),
                                transitions={'MANUAL':'MANUAL',
                                             'ALTCTL':'ALTCTL',
                                             'OFFBOARD':'OFFBOARD',
                                             'AUTO.RTL':'EMERGY',
                                             'finished':'finished'})
        smach.StateMachine.add('EMERGY', CBState(EmergyCB),
                                transitions={'MANUAL':'MANUAL',
                                             'ALTCTL':'ALTCTL',
                                             'finished':'finished'})

        # Open the container
        with sm_offb_:

            # Add states to the container
            smach.StateMachine.add('STANDBY', offb_states_[OffbState.STANDBY],
                                    transitions={'takeoff':'TAKEOFF',
                                                'AUTO.RTL':'AUTO.RTL',
                                                'ALTCTL':'ALTCTL',
                                                'MANUAL':'MANUAL'})
            smach.StateMachine.add('TAKEOFF', offb_states_[OffbState.TAKEOFF], 
                                    transitions={'done':'HOVERING',
                                                'AUTO.RTL':'AUTO.RTL',
                                                'ALTCTL':'ALTCTL',
                                                'MANUAL':'MANUAL'})
            smach.StateMachine.add('HOVERING', offb_states_[OffbState.HOVERING], 
                                    transitions={'flight':'FLIGHT',
                                                'landing':'LANDING',
                                                'AUTO.RTL':'AUTO.RTL',
                                                'ALTCTL':'ALTCTL',
                                                'MANUAL':'MANUAL'})
            smach.StateMachine.add('FLIGHT', offb_states_[OffbState.FLIGHT], 
                                    transitions={'done':'HOVERING',
                                                'AUTO.RTL':'AUTO.RTL',
                                                'ALTCTL':'ALTCTL',
                                                'MANUAL':'MANUAL'})
            smach.StateMachine.add('LANDING', offb_states_[OffbState.LANDING], 
                                    transitions={'done':'AUTO.LAND',
                                                'AUTO.RTL':'AUTO.RTL',
                                                'ALTCTL':'ALTCTL',
                                                'MANUAL':'MANUAL'})

        smach.StateMachine.add('OFFBOARD', sm_offb_,
                                transitions={'MANUAL':'MANUAL',
                                             'ALTCTL':'ALTCTL',
                                             'AUTO.LAND':'LAND',
                                             'AUTO.RTL':'EMERGY'})

    # Run state machine introspection server for smach viewer
    intro_server = IntrospectionServer('sm_viewer', sm_mode_,'/viewer')
    intro_server.start()

    # Execute SMACH plan
    outcome = sm_mode_.execute()

    '''
    ROS spin
    '''
    rospy.spin()
