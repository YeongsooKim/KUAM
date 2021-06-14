#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from enum import Enum
from math import sqrt

from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Point
from kuam_msgs.msg import AlphaEval
from uav_msgs.msg import Chat

Y_AXIS_MARGIN = [5.0, 0.5, 0.3]
BUF_SIZE = 200

class Val(Enum):
    ALPHA = 0
    D = 0
    H = 1
    X_VEL = 0
    Y_VEL = 1
    Z_VEL = 2

# Point(5.1079, -4.8057, 0.0)
# Point(5.2479, -5.0057, 0.0)

GT_TARGET_X = 5.1079
GT_TARGET_Y = -4.8057
GT_TARGET_Z = 0.0

vels_ = [0.0, 0.0, 0.0]

class Plotting:
    def __init__(self):
        self.init_time = None
        self.is_first = True

        self.time_s = []
        
        self.alpha = [[]]
        self.a_colors = ['red']
        self.a_labels = ['alpha']
        self.a_xlabel = 'time [s]'
        self.a_ylabel = 'angle [deg]'
        self.a_title = 'Alpha'
        
        self.distances = [[], []]
        self.d_colors = ['green', 'blue']
        self.d_labels = ['xy_dist', 'height']
        self.d_xlabel = 'time [s]'
        self.d_ylabel = 'distance [m]'
        self.d_title = 'Height and xy Distance'

        self.vels = [[], [], []]
        self.v_colors = ['red', 'green', 'blue']
        self.v_labels = ['x vel', 'y vel', 'z vel']
        self.v_xlabel = 'time [s]'
        self.v_ylabel = 'velocity [m/s]'
        self.v_title = 'Ego Body Velocity'

        self.list_set = [self.alpha, self.distances, self.vels]
        self.color_set = [self.a_colors, self.d_colors, self.v_colors]
        self.label_set = [self.a_labels, self.d_labels, self.v_labels]
        self.xlabel_set = [self.a_xlabel, self.d_xlabel, self.v_xlabel]
        self.ylabel_set = [self.a_ylabel, self.d_ylabel, self.v_ylabel]
        self.title_set = [self.a_title, self.d_title, self.v_title]        
        self.fig, self.axs = plt.subplots(1, 3)


    '''
    Callback functions
    '''
    def AlphaEvalCB(self, msg):
        if (msg.h_m.data == 0):
            pass
        
        else:
            if self.is_first:
                self.is_first = False
                self.init_time = rospy.Time.now()

            time_s = rospy.Time.now()
            elapse = time_s - self.init_time

            self.time_s.append(elapse.to_sec())
            self.alpha[Val.ALPHA.value].append(msg.alpha_deg.data)

            self.distances[Val.D.value].append(msg.d_m.data)
            self.distances[Val.H.value].append(msg.h_m.data)

            global vels_
            self.vels[Val.X_VEL.value].append(vels_[Val.X_VEL.value])
            self.vels[Val.Y_VEL.value].append(vels_[Val.Y_VEL.value])
            self.vels[Val.Z_VEL.value].append(vels_[Val.Z_VEL.value])
            
    def VelocityCB(self, msg):
        global vels_
        vels_[0] = msg.twist.linear.x
        vels_[1] = msg.twist.linear.y
        vels_[2] = msg.twist.linear.z

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
        for ax in self.axs:
            ax.set_xlim(0, BUF_SIZE - 1)
            ax.set_ylim(0.0, 1.0)

    def UdatePlot(self, frame):
        for type, ax in enumerate(self.axs):
            ax.clear()
            ylim = self.GetLimMinMax(self.list_set[type], Y_AXIS_MARGIN[type])
            ax.set_ylim(ylim[0], ylim[1])

            for element in range(len(self.list_set[type])):
                if (len(self.time_s) == len(self.list_set[type][element])):
                    ax.plot(self.time_s, self.list_set[type][element], color=self.color_set[type][element], label=self.label_set[type][element])

            ax.set_xlabel(self.xlabel_set[type])  # Add an x-label to the axes.
            ax.set_ylabel(self.ylabel_set[type])  # Add a y-label to the axes.
            ax.set_title(self.title_set[type])  # Add a title to the axes.            
            ax.legend()  # Add a legend.

    '''
    Util functions
    '''
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

rospy.init_node('alpha_eval')
plotting = Plotting()

vel_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, plotting.VelocityCB)
vel_sub = rospy.Subscriber('/kuam/maneuver/state_machine/alpha', AlphaEval, plotting.AlphaEvalCB)
cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 