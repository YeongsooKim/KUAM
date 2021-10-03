# -*- coding: utf-8 -*-
"""
@author: Yuseung Na  (ys.na0220@gmail.com) 
         Chansoo Kim (kchsu531@gmail.com)
"""
import numpy as np
import scipy.interpolate as inter

INTERVAL = 0.2

def LengthOfDegreeLatitude(ref_latitude_deg):
    # Convert latitude to radians
    ref_latitude_rad = float(ref_latitude_deg) * np.pi / 180.

    # Set up "Constants"
    m1 = 111132.92 # Latitude calculation term 1
    m2 = -559.82 # Latitude calculation term 2
    m3 = 1.175 # Latitude calculation term 3
    m4 = -0.0023 # Latitude calculation term 4

    # Calculate the length of a degree of latitude and longitude in meters
    return m1 + (m2 * np.cos(2 * ref_latitude_rad)) + (m3 * np.cos(4 * ref_latitude_rad)) + (m4 * np.cos(6 * ref_latitude_rad))
    
def LengthOfDegreeLongitude(ref_latitude_deg):
    # Convert latitude to radians
    ref_latitude_rad = float(ref_latitude_deg) * np.pi / 180.

    # Set up "Constants"
    p1 = 111412.84 # Longitude calculation term 1
    p2 = -93.5 # Longitude calculation term 2
    p3 = 0.118 # Longitude calculation term 3

    # Calculate the length of a degree of latitude and longitude in meters
    return (p1 * np.cos(ref_latitude_rad)) + (p2 * np.cos(3 * ref_latitude_rad)) + (p3 * np.cos(5 * ref_latitude_rad))
    
def CartesianToWgs84( geopoint_ref, enupoint ):
    # geopoint_ref: [latitude, longitude] list for origin points
    # enupoint: [east, north] list        
    latitude = geopoint_ref[0] + enupoint[1]/LengthOfDegreeLatitude(geopoint_ref[0])
    longitude = geopoint_ref[1] + enupoint[0]/LengthOfDegreeLongitude(geopoint_ref[0])
    return [latitude, longitude]

def Wgs84ToCartesian( geopoint_ref, wgspoint ):
    east = (wgspoint[1] - geopoint_ref[1]) * LengthOfDegreeLongitude(geopoint_ref[0])
    north = (wgspoint[0] - geopoint_ref[0]) * LengthOfDegreeLatitude(geopoint_ref[0])
        
    return [east, north]

def LineSplit(arclength, size=None):
    if size is not None:
        num = size
    else:
        num = int(arclength/INTERVAL)

    x = np.linspace(0, arclength, num)
    return x

def SmoothingEnu(x,y,s, size=None):
    cfg_geometry_smoothing_max_tolerance = 0.1*s
    cfg_geometry_smoothing_improve_rate = 0.1*s
    avg_tolerance = 0.1*s
    
    #Arc-length calculation

    enu_array = np.vstack((x,y))
    enu_array = enu_array.T

    arclength_array = np.zeros(x.shape)
    difference_enu_square_array = np.diff(enu_array, n=1, axis=0)**2
    distance_array = np.sqrt(np.sum(difference_enu_square_array, axis=1))
    cum_distance_array = np.cumsum(distance_array)
    arclength_array[1:] = cum_distance_array[:]

    smooth_param = avg_tolerance**2*len(arclength_array)
    spline_smooth_east = inter.UnivariateSpline (arclength_array, enu_array[:,0], s=smooth_param)
    spline_smooth_north = inter.UnivariateSpline (arclength_array, enu_array[:,1], s=smooth_param)

    if size is not None:
        norm_arclength_array = LineSplit(arclength_array[-1], size)
    else:
        norm_arclength_array = LineSplit(arclength_array[-1])
    
    smooth_east_north_array = np.zeros((len(norm_arclength_array), 2))
    smooth_east_north_array[:,0] = spline_smooth_east(norm_arclength_array)
    smooth_east_north_array[:,1] = spline_smooth_north(norm_arclength_array)
    # distance_error_array = np.sqrt(np.sum((enu_array - smooth_east_north_array)**2, axis=1))
    # max_distance_error = np.max(distance_error_array)

    # #update s until less than maximum Distance error
    # while max_distance_error > cfg_geometry_smoothing_max_tolerance:    
    #     avg_tolerance = avg_tolerance * (1. - cfg_geometry_smoothing_improve_rate)    
    #     smooth_param = avg_tolerance**2*len(arclength_array)

    #     spline_smooth_east = inter.UnivariateSpline (arclength_array, enu_array[:,0], s=smooth_param)
    #     spline_smooth_north = inter.UnivariateSpline (arclength_array, enu_array[:,1], s=smooth_param)

    #     smooth_east_north_array = np.zeros((len(norm_arclength_array), 2))
    #     smooth_east_north_array[:,0] = spline_smooth_east(norm_arclength_array)
    #     smooth_east_north_array[:,1] = spline_smooth_north(norm_arclength_array)
    #     distance_error_array = np.sqrt(np.sum((enu_array - smooth_east_north_array)**2, axis=1))
    #     max_distance_error = np.max(distance_error_array)
    
    return smooth_east_north_array
