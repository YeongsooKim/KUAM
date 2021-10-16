#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from utils import *
from enum import Enum

import tf2_geometry_msgs
import tf2_ros

# Message
from uav_msgs.msg import Chat
from geometry_msgs.msg import PoseStamped
from kuam_msgs.msg import FittingPlane

MARGIN_RATIO = 0.3
BUF_SIZE = 200

# Data structure
class Val(Enum):
    BIG_MEASURED_ANGLE = 0
    MEDIUM_MEASURED_ANGLE = 0
    SMALL_MEASURED_ANGLE = 0
    EGO_HEIGHT = 0

# Init plotting class
class Plotting:
    def __init__(self):
        self.InitPlot()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.is_init_threshold_point = [False, False, False]
        self.vertical_threshold = [0.0, 0.0, 0.0]
        self.horizontal_threshold = [0.0, 0.0, 0.0]

        self.prev_big_z_to_normal_deg = 0
        self.prev_medium_z_to_normal_deg = 0
        self.prev_small_z_to_normal_deg = 0

    def InitPlot(self):
        # Horizontal axis
        self.height_list = []
        
        # Vertical axes
        self.b = [[]]
        self.b_colors = ['black']
        self.b_labels = ['angle']
        self.b_xlabel = 'height [m]'
        self.b_ylabel = 'degree [deg]'
        self.b_title = "Big Fitting Plane Angle"

        self.m = [[]]
        self.m_colors = ['black']
        self.m_labels = ['angle']
        self.m_xlabel = 'height [m]'
        self.m_ylabel = 'degree [deg]'
        self.m_title = "Medium Fitting Plane Angle"

        self.s = [[]]
        self.s_colors = ['black']
        self.s_labels = ['angle']
        self.s_xlabel = 'height [m]'
        self.s_ylabel = 'degree [deg]'
        self.s_title = "Small Fitting Plane Angle"

        self.list_set = [self.b, self.m, self.s]
        self.color_set = [self.b_colors, self.m_colors, self.s_colors]
        self.label_set = [self.b_labels, self.m_labels, self.s_labels]
        self.xlabel_set = [self.b_xlabel, self.m_xlabel, self.s_xlabel]
        self.ylabel_set = [self.b_ylabel, self.m_ylabel, self.s_ylabel]
        self.title_set = [self.b_title, self.m_title, self.s_title]
        self.fig, self.axs = plt.subplots(1, 3, figsize=(13.5,6))

        # Init axes
        self.line = []
        for data_idx, ax in enumerate(self.axs):
            for sub_data_idx in range(len(self.list_set[data_idx])):
                line, = ax.plot([], [], lw=2, color=self.color_set[data_idx][sub_data_idx], 
                                                label=self.label_set[data_idx][sub_data_idx])
                self.line.append(line)

            ax.set_xlabel(self.xlabel_set[data_idx])  # Add an x-label to the axes.
            ax.set_ylabel(self.ylabel_set[data_idx])  # Add a y-label to the axes.
            ax.set_title(self.title_set[data_idx])  # Add a title to the axes.            
            ax.grid()
            ax.legend()  # Add a legend.

    '''
    Callback functions
    ''' 
    def EgoLocalPoseCB(self, msg):
        self.height = msg.pose.position.z
        
    def FittingPlaneCB(self, msg):
        # Append horizontal
        
        self.height_list.insert(0, self.height)
        # Append vertical

        if not msg.big_is_valid:
            self.b[Val.BIG_MEASURED_ANGLE.value].insert(0,self.prev_big_z_to_normal_deg)
        else:
            self.b[Val.BIG_MEASURED_ANGLE.value].insert(0, msg.big_z_to_normal_deg)
            self.prev_big_z_to_normal_deg = msg.big_z_to_normal_deg
            
        if not msg.medium_is_valid:
            self.m[Val.MEDIUM_MEASURED_ANGLE.value].insert(0, self.prev_medium_z_to_normal_deg)
        else:
            self.m[Val.MEDIUM_MEASURED_ANGLE.value].insert(0, msg.medium_z_to_normal_deg)
            self.prev_medium_z_to_normal_deg = msg.medium_z_to_normal_deg
            
        if not msg.small_is_valid:
            self.s[Val.SMALL_MEASURED_ANGLE.value].insert(0, self.prev_small_z_to_normal_deg)
        else:
            self.s[Val.SMALL_MEASURED_ANGLE.value].insert(0, msg.small_z_to_normal_deg)
            self.prev_small_z_to_normal_deg = msg.small_z_to_normal_deg

        if msg.big_is_valid and msg.big_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[0]:
            rospy.logwarn("big init, height: %f", self.height)
            self.is_init_threshold_point[0] = True
            self.vertical_threshold[0] = self.height
            self.horizontal_threshold[0] = msg.big_z_to_normal_deg

        if msg.medium_is_valid and msg.medium_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[1]:
            rospy.logwarn("medium init, height: %f", self.height)
            self.is_init_threshold_point[1] = True
            self.vertical_threshold[1] = self.height
            self.horizontal_threshold[1] = msg.medium_z_to_normal_deg

        if msg.small_is_valid and msg.small_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[2]:
            rospy.logwarn("small init, height: %f", self.height)
            self.is_init_threshold_point[2] = True
            self.vertical_threshold[2] = self.height
            self.horizontal_threshold[2] = msg.small_z_to_normal_deg


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
    def UdatePlot(self, frame):
        # Plot
        line_idx = 0
        for data_idx, ax in enumerate(self.axs):
    
            ylim = self.GetYLim(self.list_set[data_idx])
            xlim = self.GetXLim(self.height_list)
            ax.set_ylim(ylim[0], ylim[1])
            ax.set_xlim(xlim[0], xlim[1])


            if data_idx == 0:
                if self.is_init_threshold_point[0]:
                    ax.vlines(self.vertical_threshold[0], ylim[0], ylim[1], color=('r'), linestyle='solid', linewidth=0.5)
                    ax.hlines(self.horizontal_threshold[0], xlim[0], xlim[1], color=('r'), linestyle='solid', linewidth=0.5)

            elif data_idx == 1:
                if self.is_init_threshold_point[1]:
                    ax.vlines(self.vertical_threshold[1], ylim[0], ylim[1], color=('g'), linestyle='solid', linewidth=0.5)
                    ax.hlines(self.horizontal_threshold[1], xlim[0], xlim[1], color=('g'), linestyle='solid', linewidth=0.5)

            elif data_idx == 2:
                if self.is_init_threshold_point[2]:
                    ax.vlines(self.vertical_threshold[2], ylim[0], ylim[1], color=('b'), linestyle='solid', linewidth=0.5)
                    ax.hlines(self.horizontal_threshold[2], xlim[0], xlim[1], color=('b'), linestyle='solid', linewidth=0.5)

            for sub_data_idx in range(len(self.list_set[data_idx])):
                if len(self.height_list) == len(self.list_set[data_idx][sub_data_idx]):
                    self.line[line_idx].set_data(self.height_list, self.list_set[data_idx][sub_data_idx])

                line_idx += 1
            
        self.fig.tight_layout()
        return self.line,


    '''
    Util functions
    '''
    def GetYLim(self, lists):
        # If empty list is exist, return
        for lst in lists:
            if len(lst) == 0:
                return [0, BUF_SIZE - 1]

        # Find min max value among all list
        list_max = -sys.maxsize - 1
        list_min = sys.maxsize
        for lst in lists:
            if max(lst) > list_max:
                list_max = max(lst)

            if min(lst) < list_min:
                list_min = min(lst)

        if abs(list_max - list_min) < 0.1:
            return [-0.5, 0.5]

        # Add margin
        margin = (list_max - list_min)*MARGIN_RATIO/2.0
        return [float(list_min) - margin, float(list_max) + margin]


    def GetXLim(self, lst):
        if len(lst) == 0:
            return [0, 10]
        elif (max(lst) - min(lst)) < 10.0:
            return [max(lst) - 10, max(lst)]
        else:
            return [min(lst), max(lst)]

    
if __name__ == "__main__":
    # Init node
    rospy.init_node('node_name')
    plotting = Plotting()

    # Init subscriber
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)
    fitting_plane_sub = rospy.Subscriber("/kuam/data/aruco_tracking/fitting_plane", FittingPlane, plotting.FittingPlaneCB)
    local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, plotting.EgoLocalPoseCB)

    # Animation
    ani = FuncAnimation(plotting.fig, plotting.UdatePlot)
    plt.show(block=True) 