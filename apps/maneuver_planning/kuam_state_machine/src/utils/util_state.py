import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))
import rospy

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from utils.util_geometry import *

def GetOptimalSetpoint(setpoints, ego_geopose, ego_pose, roi_th, prev_global_nearest_sp, prev_local_nearest_sp):
    geopose, global_idx = UpdateGlobalSetpoint(setpoints.geopath.poses, ego_geopose, roi_th, prev_global_nearest_sp)
    pose, local_idx = UpdateLocalSetpoint(setpoints.pose_array.poses, ego_pose, roi_th, prev_local_nearest_sp)

    optimal_setpoint = [geopose, global_idx, pose, local_idx]
    return optimal_setpoint


def UpdateGlobalSetpoint(setpoints, ego_geopose, roi_th, prev_nearest_idx):
    # Find contained point in range of ROI
    setpoints_contained_in_roi = {}
    min_dist = 100000
    for idx, p in enumerate(setpoints):
        if idx < prev_nearest_idx:
            continue

        # p: GeoPoseStamped message
        dist = DistanceFromLatLonInMeter3D(p.pose.position, ego_geopose.position)
        if dist < roi_th:
            setpoints_contained_in_roi[idx] = dist
            # print("index: ", idx, ", dist: ", dist)
        
        if dist < min_dist:
            nearest_idx = idx
            min_dist = dist

    if len(setpoints_contained_in_roi) == 1:
        idx = next(iter(setpoints_contained_in_roi))

        if idx < (len(setpoints) - 1):
            # print("length of setpoints_contained_in_roi = 1, return idx+1")
            return setpoints[idx+1].pose, nearest_idx
        else:
            # print("length of setpoints_contained_in_roi = 1, return idx")
            return setpoints[idx].pose, nearest_idx
    elif len(setpoints_contained_in_roi) == 0:
        if nearest_idx < (len(setpoints) - 1):
            rospy.loginfo_throttle(0.1, "No proper setpoint over roi, return nearest_idx+1")
            return setpoints[nearest_idx+1].pose, nearest_idx
        else:
            rospy.loginfo_throttle(0.1, "No proper setpoint over roi, return nearest_idx")
            return setpoints[nearest_idx].pose, nearest_idx

    # Find over the nearest point
    over_nearest_idx = {}
    for idx in setpoints_contained_in_roi:
        if idx > nearest_idx:
            over_nearest_idx[idx] = setpoints_contained_in_roi[idx]

    if len(over_nearest_idx) == 0:
        # print("length of over_nearest_idx = 0")
        return setpoints[nearest_idx].pose, nearest_idx
    
    # Find the farthest point
    max_dist = 0
    for idx in over_nearest_idx:
        if over_nearest_idx[idx] > max_dist:
            farthest_idx = idx
            max_dist = over_nearest_idx[idx]
    # print("farthest index: ", farthest_idx, ", dist: ", max_dist)

    return setpoints[farthest_idx].pose, nearest_idx


def UpdateLocalSetpoint(setpoints, ego_pose, roi_th, prev_nearest_idx):
    # Find contained point in range of ROI
    setpoints_contained_in_roi = {}
    min_dist = 100000
    for idx, p in enumerate(setpoints):
        if idx < prev_nearest_idx:
            continue

        # p: GeoPoseStamped message
        dist = Distance3D(p.position, ego_pose.position)
        if dist < roi_th:
            setpoints_contained_in_roi[idx] = dist
            # print("index: ", idx, ", dist: ", dist)
        
        if dist < min_dist:
            nearest_idx = idx
            min_dist = dist

    if len(setpoints_contained_in_roi) == 1:
        idx = next(iter(setpoints_contained_in_roi))

        if idx < (len(setpoints) - 1):
            # print("length of setpoints_contained_in_roi = 1, return idx+1")
            return setpoints[idx+1], nearest_idx
        else:
            # print("length of setpoints_contained_in_roi = 1, return idx")
            return setpoints[idx], nearest_idx
    elif len(setpoints_contained_in_roi) == 0:
        if nearest_idx < (len(setpoints) - 1):
            rospy.loginfo_throttle(0.1, "No proper setpoint over roi, return nearest_idx+1")
            return setpoints[nearest_idx+1], nearest_idx
        else:
            rospy.loginfo_throttle(0.1, "No proper setpoint over roi, return nearest_idx")
            return setpoints[nearest_idx], nearest_idx

    # Find over the nearest point
    over_nearest_idx = {}
    for idx in setpoints_contained_in_roi:
        if idx > nearest_idx:
            over_nearest_idx[idx] = setpoints_contained_in_roi[idx]

    if len(over_nearest_idx) == 0:
        # print("length of over_nearest_idx = 0")
        return setpoints[nearest_idx], nearest_idx
    
    # Find the farthest point
    max_dist = 0
    for idx in over_nearest_idx:
        if over_nearest_idx[idx] > max_dist:
            farthest_idx = idx
            max_dist = over_nearest_idx[idx]
    # print("farthest index: ", farthest_idx, ", dist: ", max_dist)

    return setpoints[farthest_idx], nearest_idx


def Z_Vel(target, kp, th):
    err = target
    vel = err*kp

    if vel < -th:
        vel = -th
    elif vel > th:
        vel = th
    return vel

def XY_Vel(target, kp, th):
    err = target
    vel = err*kp

    if vel < -th:
        vel = -th
    elif vel > th:
        vel = th
    return vel

def YawRate(q, kp):
    yaw_rad = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    err = yaw_rad

    yaw_rate_rad = err*kp

    vel = quaternion_from_euler(0.0, 0.0, yaw_rate_rad)
    return vel


def YawRateRad(yaw_rad, kp):
    err = yaw_rad

    yaw_rate_rad = err*kp

    vel = quaternion_from_euler(0.0, 0.0, yaw_rate_rad)
    return vel