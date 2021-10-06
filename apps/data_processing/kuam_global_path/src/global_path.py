#!/usr/bin/env python3

from math import *
import rospy
import ros
import copy
import numpy as np
from qgc_parser import *
from utils_lane_mapping import *
from utils_tf import *

# Message
from std_msgs.msg import Bool

from kuam_msgs.msg import Qgc
from kuam_msgs.msg import QgcDatum
from kuam_msgs.msg import Completion
from kuam_msgs.msg import Waypoint
from kuam_msgs.msg import Waypoints
from kuam_msgs.srv import GlobalPathSync

from mavros_msgs.msg import HomePosition
from geographic_msgs.msg import GeoPoint
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

## Global flag
is_home_set_ = False
is_init_flight_waypoints_ = False
is_init_landing_waypoints_ = False
start_global_geoposes_gen_ = False

## Global variable
home_position_ = GeoPoint()
global_path_ = Waypoints()
global_geopose_list_ = []

## Publisher
waypoints_pub_ = None


class GlobalPathMsg:
    def __init__(self):
        self.is_init = False
        self.is_allocated = True

        self.global_path = Waypoints()

    def Modify(self, global_path):
        self.is_allocated = False

        self.global_path = global_path
        self.global_path.id += 1


'''
Process functions
'''
def ProcessCB(timer):
    global start_global_geoposes_gen_
    if start_global_geoposes_gen_:
        GenFlightWaypoints()

    if start_global_geoposes_gen_:
        GenLandingWaypoints()

    PubWaypoints()


def GenMissions():
    wp = Waypoint()
    wp.mission = "takeoff"

    global_path = Waypoints()
    global_path.id = global_path_msg.global_path.id
    global_path.waypoints.append(wp)

    global_path_msg.Modify(global_path)


def GenFlightWaypoints():
    global is_init_flight_waypoints_
    if not is_init_flight_waypoints_:
        is_init_flight_waypoints_ = True

        global_path = FlightWaypoints(qgc_datum)
        global_path_msg.Modify(global_path)

        global start_global_geoposes_gen_
        start_global_geoposes_gen_ = False


def GenLandingWaypoints():
    global is_init_landing_waypoints_
    if not is_init_landing_waypoints_:
        is_init_landing_waypoints_ = True

        global_path = LandingWaypoints()
        global_path_msg.Modify(global_path)

        rospy.logwarn("length: %d", len(global_path.waypoints))

        global start_global_geoposes_gen_
        start_global_geoposes_gen_ = False


def PubWaypoints():
    if not global_path_msg.is_allocated:
        waypoints_pub_.publish(global_path_msg.global_path)


'''
Callback functions
'''
def HomePositionCB(msg):
    global is_home_set_
    if is_home_set_ == False:
        is_home_set_ = True

        home_position_.latitude = msg.geo.latitude
        home_position_.longitude = msg.geo.longitude
        home_position_.altitude = msg.geo.altitude

        rospy.logwarn("[global_path] Home set")


def CompletionCB(msg):
    global start_global_geoposes_gen_
    global global_geopose_list_
    if msg.task == "takeoff":
        if msg.is_complete:
            start_global_geoposes_gen_ = True

            del global_geopose_list_[:]
            global_geopose_list_.append(msg.geopose)

    elif msg.task == "flight":
        rospy.logwarn("completion")
        if msg.is_complete:
            start_global_geoposes_gen_ = True

            if is_gps_only:
                rospy.logwarn("is_gps_only true")
                del global_geopose_list_[:]
                global_geopose_list_.append(msg.geopose)
            else:
                rospy.logwarn("is_gps_only false")
                del global_geopose_list_[:]
                global_geopose_list_.append(msg.geopose)

'''
Service functions
'''
def GlobalPathMsgSync():
    rospy.wait_for_service(maneuver_ns + '/mission_manager/global_path_msg_sync')
    try:
        global_path_msg_sync = rospy.ServiceProxy(maneuver_ns + '/mission_manager/global_path_msg_sync', GlobalPathSync)
        res = global_path_msg_sync(global_path_msg.is_init)
        return res.sync
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


'''
Util functions
'''
def FlightWaypoints(qgc_datum):
    poses = []

    # Get initial pose
    [east, north] = Wgs84ToCartesian([home_position_.latitude, home_position_.longitude], 
                                    [global_geopose_list_[0].position.latitude, global_geopose_list_[0].position.longitude])
    pose = Pose()
    pose.position.x = east
    pose.position.y = north
    pose.position.z = global_geopose_list_[0].position.altitude
    poses.append(pose)

    END_MIN = 2
    dt = 0.1
    z_end_m = 50
    end_time = END_MIN * 60
    num = end_time/dt
    t = np.linspace(0, end_time, int(num))
    z_ms = z_end_m/end_time

    for i, t_ in enumerate(list(t)):
        p = Pose()
        p.position.x = sqrt(t_)*cos(t_/6) + pose.position.x
        p.position.y = sqrt(t_)*sin(t_/6) + pose.position.y
        p.position.z = z_ms*t_ + pose.position.z
        poses.append(p)

    geoposes = []
    for p in poses:
        [latitude, longitude] = CartesianToWgs84([home_position_.latitude, home_position_.longitude], [p.position.x, p.position.y])
        gp = GeoPose()
        gp.position.longitude = longitude
        gp.position.latitude = latitude
        gp.position.altitude = p.position.z
        geoposes.append(gp)

    # Allocate orientation
    for i, pose in enumerate(poses):
        if len(poses) != 1:
            if (i + 1) < len(poses):
                orientation = GetOrientation(poses[i].position, poses[i+1].position);
                poses[i].orientation = orientation
                geoposes[i].orientation = orientation
            else:
                poses[i].orientation = poses[i-1].orientation;
                geoposes[i].orientation = geoposes[i-1].orientation;
        else:
            poses[i].orientation.x = 0.0;
            poses[i].orientation.y = 0.0;
            poses[i].orientation.z = 0.0;
            poses[i].orientation.w = 1.0;

            geoposes[i].orientation.x = 0.0;
            geoposes[i].orientation.y = 0.0;
            geoposes[i].orientation.z = 0.0;
            geoposes[i].orientation.w = 1.0;

    global_path = Waypoints()
    global_path.id = global_path_msg.global_path.id

    wp = Waypoint()
    wp.mission = "flight"
    wp.poses = poses
    wp.geoposes = geoposes
    global_path.waypoints.append(wp)



    # for qgc in qgc_datum.qgc_datum:
    #     # Get geopose
    #     global_geopose_list_.append(qgc.geopose)

    #     # Get pose
    #     [east, north] = Wgs84ToCartesian([home_position_.latitude, home_position_.longitude], 
    #                                     [qgc.geopose.position.latitude, qgc.geopose.position.longitude])
    #     pose = Pose()
    #     pose.position.x = east
    #     pose.position.y = north
    #     pose.position.z = qgc.geopose.position.altitude
    #     poses.append(pose)

    # # Smoothing
    # x, y, z = List2Array(poses)
    # smooth_east_north_array = SmoothingEnu(x, y, smooth_param).T
    # x_list = smooth_east_north_array.tolist()[0]
    # y_list = smooth_east_north_array.tolist()[1]
    # smooth_east_top_array = SmoothingEnu(x, z, smooth_param, len(x_list)).T
    # z_list = smooth_east_top_array.tolist()[1]
    
    # smoothed_poses = []
    # smoothed_geoposes = []
    # for i, _ in enumerate(x_list):
    #     # Get smoothed pose
    #     p = Pose()
    #     p.position.x = x_list[i]
    #     p.position.y = y_list[i]
    #     p.position.z = z_list[i]
    #     smoothed_poses.append(p)

    #     # Get smoothed geopose
    #     [latitude, longitude] = CartesianToWgs84([home_position_.latitude, home_position_.longitude], [x_list[i], y_list[i]])
    #     gp = GeoPose()
    #     gp.position.longitude = longitude
    #     gp.position.latitude = latitude
    #     gp.position.altitude = z_list[i]
    #     smoothed_geoposes.append(gp)

    # # Allocate orientation
    # for i, pose in enumerate(smoothed_poses):
    #     if len(smoothed_poses) != 1:
    #         if (i + 1) < len(smoothed_poses):
    #             orientation = GetOrientation(smoothed_poses[i].position, smoothed_poses[i+1].position);
    #             smoothed_poses[i].orientation = orientation
    #             smoothed_geoposes[i].orientation = orientation
    #         else:
    #             smoothed_poses[i].orientation = smoothed_poses[i-1].orientation;
    #             smoothed_geoposes[i].orientation = smoothed_geoposes[i-1].orientation;
    #     else:
    #         smoothed_poses[i].orientation.x = 0.0;
    #         smoothed_poses[i].orientation.y = 0.0;
    #         smoothed_poses[i].orientation.z = 0.0;
    #         smoothed_poses[i].orientation.w = 1.0;

    #         smoothed_geoposes[i].orientation.x = 0.0;
    #         smoothed_geoposes[i].orientation.y = 0.0;
    #         smoothed_geoposes[i].orientation.z = 0.0;
    #         smoothed_geoposes[i].orientation.w = 1.0;

    # global_path = Waypoints()
    # global_path.id = global_path_msg.global_path.id

    # wp = Waypoint()
    # wp.mission = "flight"
    # wp.poses = smoothed_poses
    # wp.geoposes = smoothed_geoposes
    # global_path.waypoints.append(wp)

    return global_path

def LandingWaypoints():

    if is_gps_only:
        NUM = 15
        interval = global_geopose_list_[0].position.altitude/NUM

        geoposes = []
        for i in range(NUM-1):
            p = GeoPose()
            p.position.longitude = global_geopose_list_[0].position.longitude
            p.position.latitude = global_geopose_list_[0].position.latitude
            p.position.altitude = global_geopose_list_[0].position.altitude - interval*i
            p.orientation = global_geopose_list_[0].orientation
            geoposes.append(p)

        last_p = GeoPose()
        last_p.position.longitude = global_geopose_list_[0].position.longitude
        last_p.position.latitude = global_geopose_list_[0].position.latitude
        last_p.position.altitude = 0
        last_p.orientation = global_geopose_list_[0].orientation
        geoposes.append(last_p)


        # Get initial pose
        [east, north] = Wgs84ToCartesian([home_position_.latitude, home_position_.longitude], 
                                        [global_geopose_list_[0].position.latitude, global_geopose_list_[0].position.longitude])
        rospy.logwarn("altitude: %f", global_geopose_list_[0].position.altitude)

        poses = []
        for i in range(NUM-1):
            if i == 0:
                continue
            
            p = Pose()
            p.position.x = east
            p.position.y = north
            p.position.z = global_geopose_list_[0].position.altitude - interval*i
            p.orientation = global_geopose_list_[0].orientation
            poses.append(p)
        rospy.logwarn("poses size: %d", len(poses))

        last_p = Pose()
        last_p.position.x = east
        last_p.position.y = north
        last_p.position.z = global_geopose_list_[0].position.altitude - interval*i
        last_p.orientation = global_geopose_list_[0].orientation
        poses.append(last_p)

        global_path = Waypoints()
        global_path.id = global_path_msg.global_path.id
        
        wp = Waypoint()
        wp.mission = "landing"
        wp.poses = poses
        wp.geoposes = geoposes
        global_path.waypoints.append(wp)
    
    # using gps and camera
    else:
        poses = []

        # Get initial pose
        [east, north] = Wgs84ToCartesian([home_position_.latitude, home_position_.longitude], 
                                        [global_geopose_list_[0].position.latitude, global_geopose_list_[0].position.longitude])
        pose = Pose()
        pose.position.x = east
        pose.position.y = north
        pose.position.z = global_geopose_list_[0].position.altitude
        poses.append(pose)

        global_path = Waypoints()
        global_path.id = global_path_msg.global_path.id
        
        wp = Waypoint()
        wp.mission = "landing"
        wp.poses = poses
        wp.geoposes = global_geopose_list_
        global_path.waypoints.append(wp)
    
    return global_path

def List2Array(poses):
    x_list = []
    for pose in poses:
        x = pose.position.x
        x_list.append(x)
    x_array = np.array(x_list)

    y_list = []
    for pose in poses:
        y = pose.position.y
        y_list.append(y)
    y_array = np.array(y_list)

    z_list = []
    for pose in poses:
        z = pose.position.z
        z_list.append(z)
    z_array = np.array(z_list)

    return x_array, y_array, z_array


'''
Main
'''
if __name__ == '__main__':
    rospy.init_node('global_path')
    qgc_datum = Paser('csv/waypoints.csv')

    '''
    Initialize Parameters
    '''
    freq = rospy.get_param("~process_freq")
    using_fake_gps = rospy.get_param("~using_fake_gps")
    latitude = rospy.get_param("~latitude")
    longitude = rospy.get_param("~longitude")
    altitude = rospy.get_param("~altitude")
    smooth_param = rospy.get_param("~smooth_param")
    maneuver_ns = rospy.get_param("~maneuver_ns")
    is_gps_only = rospy.get_param("~gps_only")

    '''
    Initialize ROS
    '''
    # Init subscriber
    if using_fake_gps:
        is_home_set_ = True
        home_position_.latitude = latitude
        home_position_.longitude = longitude
        home_position_.altitude = altitude
    else:
        home_position_sub = rospy.Subscriber("/mavros/home_position/home", HomePosition, HomePositionCB)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not is_home_set_:
            rospy.logwarn("waiting home set")
            rate.sleep()
    completion_sub = rospy.Subscriber(maneuver_ns + "/state_machine/completion", Completion, CompletionCB)

    # Init publisher
    waypoints_pub_ = rospy.Publisher('~waypoints', Waypoints, queue_size=10)

    # Init timer
    global_path_timer = rospy.Timer(rospy.Duration(1/freq), ProcessCB)
    
    '''
    Initialize
    '''
    global_path_msg = GlobalPathMsg ()
    global_path_msg.global_path.header.frame_id = "map"

    if GlobalPathMsgSync():
        global_path_msg.is_init = True
        rospy.logwarn("Complete global path sync")
    GenMissions()


    '''
    ROS spin
    '''
    rospy.spin()
