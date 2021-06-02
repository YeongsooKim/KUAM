#!/usr/bin/env python2.7

import rospy
import ros
from math import pi

import tf_conversions
from enum import Enum
import csv
import copy
import string

# Message
from kuam_msgs.msg import Waypoint
from kuam_msgs.msg import Waypoints
from geometry_msgs.msg import Quaternion


'''
Const value
'''
COLUMN_SIZE = 11

'''
Enum class
'''
# Init state machine state
class Col(Enum):
    Mission = 4 # QGroundControl param 1
    Yaw = 7
    Lat = 8
    Long = 9
    Alt = 10

class MissionTable(Enum):
    Flight = 0
    Takeoff = 1
    Landing = 2
    Transition = 3
    Docking = 4
    Undocking = 5

def Euler2Quat(yaw_deg):
    yaw_rad = yaw_deg*pi/180.0

    quat_msg = Quaternion()
    quat_np = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)
    quat_msg.x = quat_np[0]
    quat_msg.y = quat_np[1]
    quat_msg.z = quat_np[2]
    quat_msg.w = quat_np[3]
    
    return quat_msg


with open('csv/waypoints.csv') as f:
    reader = csv.reader(f)

    '''
    Read file
    '''
    data = []
    for index, row in enumerate(reader):
        del row[0]
        del row[0]

        if index == 1:
            data = row
    
    '''
    Gen extend waypoints
    '''
    ex_waypoints = []
    ex_waypoint = []
    for index, col in enumerate(data):
        if (index % COLUMN_SIZE == 0) and (index != 0):
            way = copy.deepcopy(ex_waypoint)
            ex_waypoints.append(way)
            del ex_waypoint[:]
        ex_waypoint.append(col)

    way = copy.deepcopy(ex_waypoint)
    ex_waypoints.append(way)
    del ex_waypoints[0] # remove launch waypoint

    '''
    Gen waypoints
    '''
    waypoints = []
    waypoint = []
    missions = []
    for ex_waypoint in ex_waypoints:
        for element_num, element in enumerate(ex_waypoint): 
            if (element_num == Col.Mission.value):
                missions.append(element)

            if (element_num == Col.Yaw.value)  or \
                (element_num == Col.Lat.value)  or \
                (element_num == Col.Long.value)  or \
                (element_num == Col.Alt.value):

                waypoint.append(element)

        way = copy.deepcopy(waypoint)
        waypoints.append(way)
        del waypoint[:]

    missions[0] = MissionTable.Takeoff.value

    '''
    Allocate mission
    '''
    for index, mission in enumerate(missions):
        if int(float(mission)) == MissionTable.Flight.value:
            waypoints[index].append(str.lower(MissionTable.Flight.name))

        elif int(float(mission)) == MissionTable.Takeoff.value:
            waypoints[index].append(str.lower(MissionTable.Takeoff.name))

        elif int(float(mission)) == MissionTable.Landing.value:
            waypoints[index].append(str.lower(MissionTable.Landing.name))

        elif int(float(mission)) == MissionTable.Transition.value:
            waypoints[index].append(str.lower(MissionTable.Transition.name))

        elif int(float(mission)) == MissionTable.Docking.value:
            waypoints[index].append(str.lower(MissionTable.Docking.name))

        elif int(float(mission)) == MissionTable.Undocking.value:
            waypoints[index].append(str.lower(MissionTable.Undocking.name))

    wps = Waypoints()
    wps.header.frame_id = "map"

    for waypoint in waypoints:
        wp = Waypoint()
        wp.geopose.orientation = Euler2Quat(float(waypoint[0]))
        wp.geopose.position.latitude = float(waypoint[1])
        wp.geopose.position.longitude = float(waypoint[2])
        wp.geopose.position.altitude = float(waypoint[3])
        wp.mission = waypoint[4]

        wps.waypoints.append(wp)

        


def ParserTimerCB(msg):
    kuam_waypoints_pub.publish(wps)

if __name__ == '__main__':
    rospy.init_node('csv_parser')

    '''
    Initialize Parameters
    '''
    # dist_thresh_m = rospy.get_param(nd_name_ + "/reached_dist_th_m")

    '''
    Initialize ROS
    '''
    nd_name_ = rospy.get_name()
    # Init subcriber
    # rospy.Subscriber("/mavros/local_position/pose", PoseStamped, EgoPoseCB)

    # Init publisher
    kuam_waypoints_pub = rospy.Publisher(nd_name_ + '/waypoints', Waypoints, queue_size=10)

    # Init timer
    parser_timer = rospy.Timer(rospy.Duration(3), ParserTimerCB)

    '''
    ROS spin
    '''
    rospy.spin()
