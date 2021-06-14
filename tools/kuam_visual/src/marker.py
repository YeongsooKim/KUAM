#!/usr/bin/env python

import rospy

# Message
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker,MarkerArray

BIG_ARC_POS = Point(6.2933, -6.5594, 0.0)
SMALL_ARC_POS = Point(6.4333, -6.7594, 0.0)

# left_top = Point(-0.48, 0.48, 0.0)
# right_top = Point(0.48, 0.48, 0.0)
# right_bt = Point(0.48, -0.48, 0.0)
# left_bt = Point(-0.48, -0.48, 0.0)
# center = Point(0.0, 0.0, 0.0)

'''
Util functions
'''
def MarkerPub():
    marker_array = MarkerArray()
    red = ColorRGBA(1.0, 0.0, 0.0, 1.0)
    green = ColorRGBA(0.0, 1.0, 0.0, 1.0)
    blue = ColorRGBA(0.0, 0.0, 1.0, 1.0)




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
    gt_marker.pose.position = BIG_ARC_POS
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
    gt_marker.color = blue
    gt_marker.color.a = 1.0
    gt_marker.pose.orientation.x = 0.0
    gt_marker.pose.orientation.y = 0.0
    gt_marker.pose.orientation.z = 0.0
    gt_marker.pose.orientation.w = 1.0
    gt_marker.pose.position = SMALL_ARC_POS
    marker_array.markers.append(gt_marker)

    # # Cube
    # gt_marker = Marker()
    # gt_marker.ns = "ground_truth"
    # gt_marker.id = 0
    # gt_marker.header.frame_id = "map"
    # gt_marker.type = gt_marker.CUBE
    # gt_marker.action = gt_marker.ADD

    # gt_marker.scale.x = 0.2
    # gt_marker.scale.y = 0.2
    # gt_marker.scale.z = 0.01
    # a = ColorRGBA()
    # gt_marker.color = blue
    # gt_marker.color.a = 0.5
    # gt_marker.pose.orientation.x = 0.0
    # gt_marker.pose.orientation.y = 0.0
    # gt_marker.pose.orientation.z = 0.0
    # gt_marker.pose.orientation.w = 1.0
    # gt_marker.pose.position = left_top
    # marker_array.markers.append(gt_marker)

    marker_pub.publish(marker_array)

def ProcessCB(timer):
    MarkerPub()

                    
if __name__ == '__main__':
    rospy.init_node('marker')
    nd_name = rospy.get_name()
    ns_name = rospy.get_namespace()

    '''
    Initialize Parameters
    '''

    '''
    Initialize ROS
    '''
    # Init publisher
    marker_pub = rospy.Publisher(nd_name + '/gt', MarkerArray, queue_size=10)

    freq = 10.0
    # Init timer
    state_machine_timer = rospy.Timer(rospy.Duration(1/freq), ProcessCB)

    '''
    ROS spin
    '''
    rospy.spin()
