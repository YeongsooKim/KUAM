#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy
import math
import copy

from kuam_msgs.msg import TargetStateArray

Y_AXIS_MARGIN = 0.3
BUF_SIZE = 200

class Plotting:
    def __init__(self):
        self.wof = []
        self.maf = []
        self.emaf = []

        self.fig, self.ax = plt.subplots()
        
        self.ax.set_xlabel('step')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist [m]')  # Add a y-label to the axes.
        self.ax.set_title("Distance")  # Add a title to the axes.

    def PlotInit(self):
        self.ax.set_ylim(0, BUF_SIZE - 1)
        self.ax.set_ylim(0.0, 1.0)

    def TargetStatesCallback(self, target_states):
        wof_pos = target_states.states[0].pose.position
        maf_pos = target_states.states[1].pose.position
        emaf_pos = target_states.states[2].pose.position

        self.wof.append(self.Distance(wof_pos))
        self.maf.append(self.Distance(maf_pos))
        self.emaf.append(self.Distance(emaf_pos))

    def UdatePlot(self, frame):
        self.ax.clear()
        
        wof_dists = copy.deepcopy(self.wof)
        maf_dists = copy.deepcopy(self.maf)
        emaf_dists = copy.deepcopy(self.emaf)

        xlim, ylim = self.GetLimMinMax(wof_dists, maf_dists, emaf_dists)

        self.ax.set_xlabel('step')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist [m]')  # Add a y-label to the axes.
        self.ax.set_title("Distance")  # Add a title to the axes.
        self.ax.set_xlim(xlim[0], xlim[1])
        self.ax.set_ylim(ylim[0], ylim[1])

        length = len(wof_dists)
        steps = np.linspace(0, length - 1 , length)        
        if len(steps) == len(wof_dists):
            self.ax.plot(steps, wof_dists, color="green", label='w/o filter')  # Plot some data on the axes.

        length = len(maf_dists)
        steps = np.linspace(0, length - 1 , length)
        if len(steps) == len(maf_dists):
            self.ax.plot(steps, maf_dists, color="red", label='mov avg filter')  # Plot some data on the axes.

        length = len(emaf_dists)
        steps = np.linspace(0, length - 1 , length)
        if len(steps) == len(emaf_dists):
            self.ax.plot(steps, emaf_dists, color="yellow", label='exp mov avg filter')  # Plot some data on the axes.
        self.ax.legend()  # Add a legend.

    def Distance(self, point):
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        return distance

    def GetLimMinMax(self, list1, list2, list3):
        xlim = [0, BUF_SIZE - 1]
        len1 = len(list1)
        len2 = len(list2)
        len3 = len(list3)

        if (len1 > BUF_SIZE) or (len2 > BUF_SIZE) or (len3 > BUF_SIZE):
            max_len = len1
            if len2 > max_len:
                max_len = len2
            if len3 > max_len:
                max_len = len3

            xlim = [max_len - BUF_SIZE, max_len - 1]
        
            list1_max = max(list1[-BUF_SIZE:])
            list2_max = max(list2[-BUF_SIZE:])
            list3_max = max(list3[-BUF_SIZE:])

            list1_min = min(list1[-BUF_SIZE:])
            list2_min = min(list2[-BUF_SIZE:])
            list3_min = min(list3[-BUF_SIZE:])
        else:
            list1_max = max(list1)
            list2_max = max(list2)
            list3_max = max(list3)

            list1_min = min(list1)
            list2_min = min(list2)
            list3_min = min(list3)

        list_max = list1_max
        if list2_max > list_max:
            list_max = list2_max
        if list3_max > list_max:
            list_max = list3_max
    
        list_min = list1_min
        if list2_min > list_min:
            list_min = list2_min
        if list3_min > list_min:
            list_min = list3_min

        ylim = [list_min - Y_AXIS_MARGIN, list_max + Y_AXIS_MARGIN]

        return xlim, ylim


rospy.init_node('eval')
plotting = Plotting()
sub = rospy.Subscriber('/perception/aruco_tracking/target_states', TargetStateArray, plotting.TargetStatesCallback)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 