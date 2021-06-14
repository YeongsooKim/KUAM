#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy
import math
import copy

from kuam_msgs.msg import ArucoState
from geometry_msgs.msg import Point

Y_AXIS_MARGIN = 0.3
BUF_SIZE = 200

def Distance3D(point1, point2):
    delta_x = point1.x - point2.x
    delta_y = point1.y - point2.y
    delta_z = point1.z - point2.z
    
    distance_m = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2))
    return distance_m

class Plotting:
    def __init__(self):
        self.is_detected = []

        self.fig, self.ax = plt.subplots()
        
        self.ax.set_xlabel('step')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist error [m]')  # Add a y-label to the axes.
        self.ax.set_title("Distance error")  # Add a title to the axes.

    def PlotInit(self):
        self.ax.set_ylim(0, BUF_SIZE - 1)
        self.ax.set_ylim(0.0, 1.0)

    def ErrorCB(self, msg):
        is_detected = msg.is_detected

        if is_detected == True:
            val = 1
        else:
            val = 0

        self.is_detected.append(val)

    def UdatePlot(self, frame):
        self.ax.clear()
        
        vals = copy.deepcopy(self.is_detected)

        xlim, ylim = self.GetLimMinMax(vals)

        self.ax.set_xlabel('step')  # Add an x-label to the axes.
        self.ax.set_ylabel('Detected')  # Add a y-label to the axes.
        self.ax.set_title("Object Detected")  # Add a title to the axes.
        self.ax.set_xlim(xlim[0], xlim[1])
        self.ax.set_ylim(ylim[0], ylim[1])

        length = len(vals)
        steps = np.linspace(0, length - 1 , length)        
        if len(steps) == len(vals):
            self.ax.plot(steps, vals, color="red", label='detected val')  # Plot some data on the axes.
        self.ax.legend()  # Add a legend.

    def GetLimMinMax(self, list1):
        xlim = [0, BUF_SIZE - 1]
        ylim = [0, BUF_SIZE - 1]
        len1 = len(list1)

        if len1 == 0:
            return xlim, ylim
        else:
            if len1 > BUF_SIZE:
                max_len = len1

                xlim = [0, max_len - 1]
            
                list1_max = max(list1[-BUF_SIZE:])
                list1_min = min(list1[-BUF_SIZE:])
            else:
                list1_max = max(list1)
                list1_min = min(list1)

            list_max = list1_max
            list_min = list1_min

            ylim = [list_min - Y_AXIS_MARGIN, list_max + Y_AXIS_MARGIN]
            ylim = [-0.5, 1.5]

            return xlim, ylim


rospy.init_node('eval')
plotting = Plotting()
sub = rospy.Subscriber('/aruco_tracking/target_state', ArucoState, plotting.ErrorCB)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 