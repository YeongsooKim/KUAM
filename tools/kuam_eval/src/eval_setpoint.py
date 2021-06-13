#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import rospy
import math
import copy
import sys

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from uav_msgs.msg import Chat

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
        self.time_s = []
        self.alts = []
        self.alt = 0.0
        self.init_time = None
        self.is_first_error = True
        self.run_subscribing = True

        self.fig, self.ax = plt.subplots()
        
        self.ax.set_xlabel('time [s]')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist error [m]')  # Add a y-label to the axes.
        self.ax.set_title("Error between Marker Ground Truth and Setpoint")  # Add a title to the axes.

    def PlotInit(self):
        self.ax.set_xlim(0, BUF_SIZE - 1)
        self.ax.set_ylim(0.0, 1.0)

    def ErrorCB(self, point):
        if self.is_first_error:
            self.is_first_error = False
            self.init_time = rospy.Time.now()

        zero_point = Point()
        dist_err = Distance3D(zero_point, point)
        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        self.x_err.append(point.x)
        self.y_err.append(point.y)
        self.z_err.append(point.z)
        self.dist_err.append(dist_err)
        self.time_s.append(elapse.to_sec())
        self.alts.append(self.alt)

        print("dist: %.3f" %dist_err, "x_err: %.3f" %point.x, "y_err: %.3f" %point.y, "z_err: %.3f" %point.z, "true alt: %.3f" %self.alt)

    def EgoLocalPoseCB(self, msg):
        self.alt = msg.pose.position.z

    def ChatCB(self, msg):
        cmd = msg.msg
        if cmd == 'stop':
            ani.event_source.stop()
            rospy.loginfo("[eval] stop plotting")
        elif cmd == 'play':
            ani.event_source.start()
            rospy.loginfo("[eval] play plotting")


    def UdatePlot(self, frame):
        self.ax.clear()
        
        x_errs = copy.deepcopy(self.x_err)
        y_errs = copy.deepcopy(self.y_err)
        z_errs = copy.deepcopy(self.z_err)
        dist_err = copy.deepcopy(self.dist_err)
        time_s = copy.deepcopy(self.time_s)
        alts = copy.deepcopy(self.alts)

        ylim = self.GetLimMinMax(x_errs, y_errs, z_errs, self.alts)

        self.ax.set_xlabel('time [s]')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist error [m]')  # Add a y-label to the axes.
        self.ax.set_title("Error between Marker Ground Truth and Setpoint")  # Add a title to the axes.
        self.ax.set_ylim(ylim[0], ylim[1])

        if len(time_s) == len(alts):
            self.ax.plot(time_s, alts, color="black", label='gt of drone altitude [m]')  # Plot some data on the axes.

        if len(time_s) == len(x_errs):
            self.ax.plot(time_s, x_errs, color="red", label='x error [m]')  # Plot some data on the axes.

        if len(time_s) == len(y_errs):
            self.ax.plot(time_s, y_errs, color="green", label='y error [m]')  # Plot some data on the axes.

        if len(time_s) == len(z_errs):
            self.ax.plot(time_s, z_errs, color="blue", label='z error [m]')  # Plot some data on the axes.

        if len(time_s) == len(dist_err):
            self.ax.plot(time_s, dist_err, color="magenta", label='dist error [m]')  # Plot some data on the axes.
        self.ax.legend()  # Add a legend.

    def Distance(self, point):
        distance = math.sqrt(point.x**2 + point.y**2 + point.z**2)
        return distance

    def GetLimMinMax(self, list1, list2, list3, list4):
        ylim = [0, BUF_SIZE - 1]
        lengths = [len(list1), len(list2), len(list3), len(list4)]
        lists = [list1, list2, list3, list4]

        # If empty list is exist, return
        for leng in lengths:
            if leng == 0:
                return ylim

        # Find min max value
        if (min(lengths) > BUF_SIZE):
            list_max = -sys.maxsize - 1
            for list in lists:
                if max(list[-BUF_SIZE:]) > list_max:
                    list_max = max(list[-BUF_SIZE:])

            list_min = sys.maxsize
            for list in lists:
                if min(list[-BUF_SIZE:]) < list_min:
                    list_min = min(list[-BUF_SIZE:])
        else:
            list_max = -sys.maxsize - 1
            for list in lists:
                if max(list) > list_max:
                    list_max = max(list)

            list_min = sys.maxsize
            for list in lists:
                if min(list) < list_min:
                    list_min = min(list)

        ylim = [list_min - Y_AXIS_MARGIN, list_max + Y_AXIS_MARGIN]

        return ylim



rospy.init_node('eval')
plotting = Plotting()

error_sub = rospy.Subscriber('/kuam/control/payload_cmd/error', Point, plotting.ErrorCB)
ego_local_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, plotting.EgoLocalPoseCB)
cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 