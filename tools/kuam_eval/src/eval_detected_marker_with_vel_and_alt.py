#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from enum import Enum
from math import sqrt

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from std_msgs.msg import Int16
from uav_msgs.msg import Chat

Y_AXIS_MARGIN = [1.0, 1.0, 2.0, 0.15]
BUF_SIZE = 200

BIG_MARKER = [6.2933, -6.5594, 0.0]
SMALL_MARKER = [6.4333, -6.7594, 0.0]

class Val(Enum):
    TARGET = 0
    ALT = 0
    VEL_X = 0
    VEL_Y = 1
    VEL_Z = 2
    RMSE_X = 0
    RMSE_Y = 1

ego_pos_ = Point()
target_ = 0

class Plotting:
    def __init__(self):
        self.init_time = None
        self.is_first = True
        self.err_squar_sums = [0.0, 0.0]
        self.err_cnt = 0
        self.big_rmse_init = False
        self.small_rmse_init = False

        self.time_s = []
        
        self.vels = [[], [], []]
        self.v_colors = ['red', 'green', 'blue']
        self.v_labels = ['x vel', 'y vel', 'z vel']
        self.v_xlabel = 'time [s]'
        self.v_ylabel = 'velocity [m/s]'
        self.v_title = "Ego Body Velocity"

        self.target = [[]]
        self.t_colors = ['red']
        self.t_labels = ['target']
        self.t_xlabel = 'time [s]'
        self.t_ylabel = 'marker id + 1 [ ]'
        self.t_title = 'Target Marker ID'
        
        self.heights = [[]]
        self.h_colors = ['green']
        self.h_labels = ['height']
        self.h_xlabel = 'time [s]'
        self.h_ylabel = 'distance [m]'
        self.h_title = 'Ego Vehicle Height'

        self.rmses = [[], []]
        self.r_colors = ['red', 'green']
        self.r_labels = ['x rmse', 'y rmse']
        self.r_xlabel = 'time [s]'
        self.r_ylabel = 'rmse [m]'
        self.r_title = "Error between Marker Ground Truth and Ego Pos"

        self.list_set = [self.heights, self.vels, self.target, self.rmses]
        self.color_set = [self.h_colors, self.v_colors, self.t_colors, self.r_colors]
        self.label_set = [self.h_labels, self.v_labels, self.t_labels, self.r_labels]
        self.xlabel_set = [self.h_xlabel, self.v_xlabel, self.t_xlabel, self.r_xlabel]
        self.ylabel_set = [self.h_ylabel, self.v_ylabel, self.t_ylabel, self.r_ylabel]
        self.title_set = [self.h_title, self.v_title, self.t_title, self.r_title]
        self.fig, self.axs = plt.subplots(2, 2)

    '''
    Callback functions
    ''' 
    def VelocityCB(self, msg):
        if self.is_first:
            self.is_first = False
            self.init_time = rospy.Time.now()

        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        self.time_s.append(elapse.to_sec())
        self.vels[Val.VEL_X.value].append(msg.twist.linear.x)
        self.vels[Val.VEL_Y.value].append(msg.twist.linear.y)
        self.vels[Val.VEL_Z.value].append(msg.twist.linear.z)

        global target_
        self.target[Val.TARGET.value].append(target_)

        global ego_pos_
        self.heights[Val.ALT.value].append(ego_pos_.z)

        if target_ == 1:
            if self.big_rmse_init == False:
                self.big_rmse_init = True
                self.err_cnt = 0
                self.err_squar_sums = [0.0, 0.0]

            err_x = BIG_MARKER[0] - ego_pos_.x
            err_y = BIG_MARKER[1] - ego_pos_.y
            rmses = self.RMSE([err_x, err_y])

            self.rmses[Val.RMSE_X.value].append(rmses[0])
            self.rmses[Val.RMSE_Y.value].append(rmses[1])

        elif target_ == 2:
            if self.small_rmse_init == False:
                self.small_rmse_init = True
                self.err_cnt = 0
                self.err_squar_sums = [0.0, 0.0]

            err_x = SMALL_MARKER[0] - ego_pos_.x
            err_y = SMALL_MARKER[1] - ego_pos_.y
            rmses = self.RMSE([err_x, err_y])

            self.rmses[Val.RMSE_X.value].append(rmses[0])
            self.rmses[Val.RMSE_Y.value].append(rmses[1])
        else:
            self.rmses[Val.RMSE_X.value].append(0.0)
            self.rmses[Val.RMSE_Y.value].append(0.0)
            
    def EgoLocalPoseCB(self, msg):
        global ego_pos_
        ego_pos_.x = msg.pose.position.x
        ego_pos_.y = msg.pose.position.y
        ego_pos_.z = msg.pose.position.z

    def TargetMarkerCB(self, msg):
        global target_
        target_ = msg.data + 1

    def ChatCB(self, msg):
        cmd = msg.msg
        if cmd == 'pause':
            ani.event_source.stop()
            rospy.loginfo("[eval] pause plotting")
        elif cmd == 'play':
            ani.event_source.start()
            rospy.loginfo("[eval] play plotting")
 
    '''
    Plot
    '''
    def PlotInit(self):
        pass
        # for ax in self.axs:
        #     ax.set_xlim(0, BUF_SIZE - 1)
        #     ax.set_ylim(0.0, 1.0)

    def UdatePlot(self, frame):
        type = 0
        for i in range(len(self.axs)):
            for j in range(len(self.axs[i])):
                self.axs[i][j].clear()
                ylim = self.GetLimMinMax(self.list_set[type], Y_AXIS_MARGIN[type])
                self.axs[i][j].set_ylim(ylim[0], ylim[1])
                
                for element in range(len(self.list_set[type])):
                    if (len(self.time_s) == len(self.list_set[type][element])):
                        self.axs[i][j].plot(self.time_s, self.list_set[type][element], color=self.color_set[type][element], label=self.label_set[type][element])

                self.axs[i][j].set_xlabel(self.xlabel_set[type])  # Add an x-label to the axes.
                self.axs[i][j].set_ylabel(self.ylabel_set[type])  # Add a y-label to the axes.
                self.axs[i][j].set_title(self.title_set[type])  # Add a title to the axes.            
                self.axs[i][j].legend()  # Add a legend.

                type += 1

    '''
    Util functions
    '''
    def RMSE(self, errs):
        self.err_cnt += 1

        rmses = []
        for i, err in enumerate(errs):
            self.err_squar_sums[i] += pow(err,2)
        
            rmse = sqrt(self.err_squar_sums[i]/self.err_cnt)
            rmses.append(rmse)

        return rmses

    def GetLimMinMax(self, lists, margin):
        ylim = [0, BUF_SIZE - 1]
        lengths = []
        for lst in lists:
            lengths.append(len(lst))

        # If empty list is exist, return
        for leng in lengths:
            if leng == 0:
                return ylim

        # Find min max value
        # if (min(lengths) > BUF_SIZE):
        #     list_max = -sys.maxsize - 1
        #     for list in lists:
        #         if max(list[-BUF_SIZE:]) > list_max:
        #             list_max = max(list[-BUF_SIZE:])

        #     list_min = sys.maxsize
        #     for list in lists:
        #         if min(list[-BUF_SIZE:]) < list_min:
        #             list_min = min(list[-BUF_SIZE:])
        # else:
        list_max = -sys.maxsize - 1
        for lst in lists:
            if max(lst) > list_max:
                list_max = max(lst)

        list_min = sys.maxsize
        for lst in lists:
            if min(lst) < list_min:
                list_min = min(lst)

        ylim = [float(list_min) - margin, float(list_max) + margin]

        return ylim

rospy.init_node('multi_marker_eval')
plotting = Plotting()

vel_sub = rospy.Subscriber('/kuam/data/aruco_tracking/target_marker', Int16, plotting.TargetMarkerCB)
ego_local_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, plotting.EgoLocalPoseCB)
vel_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, plotting.VelocityCB)
cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 