import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils.util_geometry import *

def UpdateSetpoint(setpoints, ego_geopose, ego_pose, roi_th):
    geopose = UpdateGlobalSetpoint(setpoints.geopath.poses, ego_geopose, roi_th)
    pose = UpdateLocalSetpoint(setpoints.pose_array.poses, ego_pose, roi_th)

    return geopose, pose


def UpdateGlobalSetpoint(setpoints, ego_geopose, roi_th):
    # Find contained point in range of ROI
    contained_roi_idx = {}
    min_dist = 100000
    for idx, p in enumerate(setpoints):
        # p: GeoPoseStamped message
        dist = DistanceFromLatLonInMeter3D(p.pose.position, ego_geopose.position)
        if dist < roi_th:
            contained_roi_idx[idx] = dist
            # print("index: ", idx, ", dist: ", dist)
        
        if dist < min_dist:
            nearest_idx = idx
            min_dist = dist

    if len(contained_roi_idx) == 1:
        idx = next(iter(contained_roi_idx))

        if idx < (len(setpoints) - 1):
            # print("length of contained_roi_idx = 1, return idx+1")
            return setpoints[idx+1].pose
        else:
            # print("length of contained_roi_idx = 1, return idx")
            return setpoints[idx].pose
    elif len(contained_roi_idx) == 0:
        print("No proper setpoint. over roi")
        return ego_geopose
        
        if nearest_idx < (len(setpoints) - 1):
            # print("length of contained_roi_idx = 0, return nearest_idx+1")
            return setpoints[nearest_idx+1].pose
        else:
            # print("length of contained_roi_idx = 0, return nearest_idx")
            return setpoints[nearest_idx].pose

    # Find over the nearest point
    over_nearest_idx = {}
    for idx in contained_roi_idx:
        if idx > nearest_idx:
            over_nearest_idx[idx] = contained_roi_idx[idx]

    if len(over_nearest_idx) == 0:
        # print("length of over_nearest_idx = 0")
        return setpoints[nearest_idx].pose
    
    # Find the farthest point
    max_dist = 0
    for idx in over_nearest_idx:
        if over_nearest_idx[idx] > max_dist:
            farthest_idx = idx
            max_dist = over_nearest_idx[idx]
    # print("farthest index: ", farthest_idx, ", dist: ", max_dist)

    return setpoints[farthest_idx].pose


def UpdateLocalSetpoint(setpoints, ego_pose, roi_th):
    # Find contained point in range of ROI
    contained_roi_idx = {}
    min_dist = 100000
    for idx, p in enumerate(setpoints):
        # p: GeoPoseStamped message
        dist = Distance3D(p.position, ego_pose.position)
        if dist < roi_th:
            contained_roi_idx[idx] = dist
            # print("index: ", idx, ", dist: ", dist)
        
        if dist < min_dist:
            nearest_idx = idx
            min_dist = dist

    if len(contained_roi_idx) == 1:
        idx = next(iter(contained_roi_idx))

        if idx < (len(setpoints) - 1):
            # print("length of contained_roi_idx = 1, return idx+1")
            return setpoints[idx+1]
        else:
            # print("length of contained_roi_idx = 1, return idx")
            return setpoints[idx]
    elif len(contained_roi_idx) == 0:
        print("No proper setpoint. over roi")
        return ego_pose
        
        if nearest_idx < (len(setpoints) - 1):
            # print("length of contained_roi_idx = 0, return nearest_idx+1")
            return setpoints[nearest_idx+1]
        else:
            # print("length of contained_roi_idx = 0, return nearest_idx")
            return setpoints[nearest_idx]

    # Find over the nearest point
    over_nearest_idx = {}
    for idx in contained_roi_idx:
        if idx > nearest_idx:
            over_nearest_idx[idx] = contained_roi_idx[idx]

    if len(over_nearest_idx) == 0:
        # print("length of over_nearest_idx = 0")
        return setpoints[nearest_idx]
    
    # Find the farthest point
    max_dist = 0
    for idx in over_nearest_idx:
        if over_nearest_idx[idx] > max_dist:
            farthest_idx = idx
            max_dist = over_nearest_idx[idx]
    # print("farthest index: ", farthest_idx, ", dist: ", max_dist)

    return setpoints[farthest_idx]