#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from math import sqrt
import sys

from geometry_msgs.msg import PoseStamped
from uav_msgs.msg import Chat

Y_AXIS_MARGIN = 0.3
BUF_SIZE = 200

GT_TARGET_X = 0.0
GT_TARGET_Y = 0.0
GT_TARGET_Z = 0.0

class Plotting:
    def __init__(self):
        self.err_squar_sums = [0.0, 0.0, 0.0]
        self.rmses = [[], [], []]
        self.colors = ['red', 'green', 'blue']
        self.labels = ['x rmse [m]', 'y rmse [m]', 'z rmse [m]']
        self.time_s = []
        self.err_cnt = 0
        self.init_time = None
        self.is_first = True

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('time [s]')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist error [m]')  # Add a y-label to the axes.
        self.ax.set_title("Error between Marker Ground Truth and Detected Target")  # Add a title to the axes.


    def PlotInit(self):
        self.ax.set_xlim(0, BUF_SIZE - 1)
        self.ax.set_ylim(0.0, 1.0)


    def RMSE(self, errs):
        self.err_cnt += 1

        for i, err in enumerate(errs):
            self.err_squar_sums[i] += pow(err,2)
        
        for i in range(len(self.rmses)):
            rmse = sqrt(self.err_squar_sums[i]/self.err_cnt)
            self.rmses[i].append(rmse)


    def TargetPoseCB(self, msg):
        if self.is_first:
            self.is_first = False
            self.init_time = rospy.Time.now()

        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        err_x = GT_TARGET_X - msg.pose.position.x
        err_y = GT_TARGET_Y - msg.pose.position.y
        err_z = GT_TARGET_Z - msg.pose.position.z

        self.time_s.append(elapse.to_sec())
        self.RMSE([err_x, err_y, err_z])
        

    def ChatCB(self, msg):
        cmd = msg.msg
        if cmd == 'pause':
            ani.event_source.stop()
            rospy.loginfo("[eval] pause plotting")
        elif cmd == 'play':
            ani.event_source.start()
            rospy.loginfo("[eval] play plotting")


    def UdatePlot(self, frame):
        self.ax.clear()
        ylim = self.GetLimMinMax()

        self.ax.set_xlabel('time [s]')  # Add an x-label to the axes.
        self.ax.set_ylabel('dist error [m]')  # Add a y-label to the axes.
        self.ax.set_title("Error between Marker Ground Truth and Detected Target")  # Add a title to the axes.
        self.ax.set_ylim(ylim[0], ylim[1])

        for i in range(len(self.rmses)):
            if (len(self.time_s) == len(self.rmses[i])):
                self.ax.plot(self.time_s, self.rmses[i], color=self.colors[i], label=self.labels[i])
            
        self.ax.legend()  # Add a legend.


    def GetLimMinMax(self):
        ylim = [0, BUF_SIZE - 1]
        lengths = []
        for rmse in self.rmses:
            lengths.append(len(rmse))

        # If empty list is exist, return
        for leng in lengths:
            if leng == 0:
                return ylim

        # Find min max value
        # if (min(lengths) > BUF_SIZE):
        #     list_max = -sys.maxsize - 1
        #     for list in self.rmses:
        #         if max(list[-BUF_SIZE:]) > list_max:
        #             list_max = max(list[-BUF_SIZE:])

        #     list_min = sys.maxsize
        #     for list in self.rmses:
        #         if min(list[-BUF_SIZE:]) < list_min:
        #             list_min = min(list[-BUF_SIZE:])
        # else:
        list_max = -sys.maxsize - 1
        for list in self.rmses:
            if max(list) > list_max:
                list_max = max(list)

        list_min = sys.maxsize
        for list in self.rmses:
            if min(list) < list_min:
                list_min = min(list)

        ylim = [list_min - Y_AXIS_MARGIN, list_max + Y_AXIS_MARGIN]

        return ylim


rospy.init_node('target_pose_eval')
plotting = Plotting()

target_pose_sub = rospy.Subscriber('/kuam/maneuver/state_machine/target_pose', PoseStamped, plotting.TargetPoseCB)
cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 