import os
import sys
sys.path.append(os.path.dirname(os.path.abspath(os.path.dirname(__file__))))

from utils.util_geometry import *

def IsReached(p1, p2, th):
    dist = DistanceFromLatLonInMeter3D(p1, p2)
    if (dist < th):
        return True
    else: 
        return False

def UpdateSetpointPose(setpoints, ego_geopose, roi_th):

    # Find contained point in range of ROI
    contained_roi_idx = {}
    for i, p in enumerate(setpoints):
        # p: GeoPoseStamped message
        dist = DistanceFromLatLonInMeter3D(p.pose.position, ego_geopose.position)
        if dist < roi_th:
            contained_roi_idx[i] = dist
            # print("index: ", i, ", dist: ", dist)

    if len(contained_roi_idx) == 1:
        idx = next(iter(contained_roi_idx))
        # print("length of contained_roi_idx = 1")
        return setpoints[idx].pose

    # Find the nearest point with ego geopose
    min_dist = 100000
    for idx in contained_roi_idx:
        if contained_roi_idx[idx] < min_dist:
            nearest_idx = idx
            min_dist = contained_roi_idx[idx]
    # print("nearest index: ", nearest_idx, ", dist: ", min_dist)

    # Find over the nearest point
    over_nearest_idx = {}
    keys = list(contained_roi_idx.keys())
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