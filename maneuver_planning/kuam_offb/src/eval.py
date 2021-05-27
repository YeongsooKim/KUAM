#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy
import math
import copy

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
        self.x_err = []
        self.y_err = []
        self.z_err = []
        self.dist_err = []

        self.fig, self.ax = plt.subplots()
        
        self.ax.set_xlabel('step')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist error [m]')  # Add a y-label to the axes.
        self.ax.set_title("Distance error")  # Add a title to the axes.

    def PlotInit(self):
        self.ax.set_ylim(0, BUF_SIZE - 1)
        self.ax.set_ylim(0.0, 1.0)

    def ErrorCB(self, point):
        zero_point = Point()
        dist_err = Distance3D(zero_point, point)
        x_err = point.x
        y_err = point.y
        z_err = point.z

        print("dist: %.3f" %dist_err, "x_err: %.3f" %x_err, "y_err: %.3f" %y_err, "z_err: %.3f" %z_err)

        self.x_err.append(x_err)
        self.y_err.append(y_err)
        self.z_err.append(z_err)
        self.dist_err.append(dist_err)

    def UdatePlot(self, frame):
        self.ax.clear()
        
        x_errs = copy.deepcopy(self.x_err)
        y_errs = copy.deepcopy(self.y_err)
        z_errs = copy.deepcopy(self.z_err)
        dist_err = copy.deepcopy(self.dist_err)

        xlim, ylim = self.GetLimMinMax(x_errs, y_errs, z_errs)

        self.ax.set_xlabel('step')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist error [m]')  # Add a y-label to the axes.
        self.ax.set_title("Distance error")  # Add a title to the axes.
        self.ax.set_xlim(xlim[0], xlim[1])
        self.ax.set_ylim(ylim[0], ylim[1])

        length = len(x_errs)
        steps = np.linspace(0, length - 1 , length)        
        if len(steps) == len(x_errs):
            self.ax.plot(steps, x_errs, color="green", label='x error [m]')  # Plot some data on the axes.

        length = len(y_errs)
        steps = np.linspace(0, length - 1 , length)
        if len(steps) == len(y_errs):
            self.ax.plot(steps, y_errs, color="red", label='y error [m]')  # Plot some data on the axes.

        length = len(z_errs)
        steps = np.linspace(0, length - 1 , length)
        if len(steps) == len(z_errs):
            self.ax.plot(steps, z_errs, color="blue", label='z error [m]')  # Plot some data on the axes.

        length = len(dist_err)
        steps = np.linspace(0, length - 1 , length)
        if len(steps) == len(dist_err):
            self.ax.plot(steps, dist_err, color="magenta", label='dist error [m]')  # Plot some data on the axes.
        self.ax.legend()  # Add a legend.

    def Distance(self, point):
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        return distance

    def GetLimMinMax(self, list1, list2, list3):
        xlim = [0, BUF_SIZE - 1]
        ylim = [0, BUF_SIZE - 1]
        len1 = len(list1)
        len2 = len(list2)
        len3 = len(list3)

        if len1 == 0:
            return xlim, ylim
        else:
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
sub = rospy.Subscriber('/kuam/control/offb/error', Point, plotting.ErrorCB)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 