#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from utils import *
from enum import Enum
import copy

import tf2_geometry_msgs
import tf2_ros

# Message
from uav_msgs.msg import Chat
from kuam_msgs.msg import ArucoState
from kuam_msgs.msg import ArucoStates
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose

MARGIN_RATIO = 0.3
BUF_SIZE = 400

# Data structure
class Val(Enum):
    MARKER_0 = 0
    MARKER_1 = 0
    MARKER_2 = 0
    MARKER_3 = 0
    MARKER_4 = 0
    MARKER_5 = 0

# Init plotting class
class Plotting:
    def __init__(self):
        self.InitPlot()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.height = 0
        self.prev_freq_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def InitPlot(self):
        # Horizontal axis
        self.step_list = []
        self.step = 0
        
        # Vertical axes
        self.m0 = [[]]
        self.m0_colors = ['black']
        self.m0_labels = ['frequency degree']
        self.m0_xlabel = 'height [m]'
        self.m0_ylabel = 'frequency degree []'
        self.m0_title = "Marker 0 Freq Deg"

        self.m1 = [[]]
        self.m1_colors = ['black']
        self.m1_labels = ['frequency degree']
        self.m1_xlabel = 'height [m]'
        self.m1_ylabel = 'frequency degree []'
        self.m1_title = "Marker 1 Freq Deg"

        self.m2 = [[]]
        self.m2_colors = ['black']
        self.m2_labels = ['frequency degree']
        self.m2_xlabel = 'height [m]'
        self.m2_ylabel = 'frequency degree []'
        self.m2_title = "Marker 2 Freq Deg"

        self.m3 = [[]]
        self.m3_colors = ['black']
        self.m3_labels = ['frequency degree']
        self.m3_xlabel = 'height [m]'
        self.m3_ylabel = 'frequency degree []'
        self.m3_title = "Marker 3 Freq Deg"

        self.m4 = [[]]
        self.m4_colors = ['black']
        self.m4_labels = ['frequency degree']
        self.m4_xlabel = 'height [m]'
        self.m4_ylabel = 'frequency degree []'
        self.m4_title = "Marker 4 Freq Deg"

        self.m5 = [[]]
        self.m5_colors = ['black']
        self.m5_labels = ['frequency degree']
        self.m5_xlabel = 'height [m]'
        self.m5_ylabel = 'frequency degree []'
        self.m5_title = "Marker 5 Freq Deg"

        self.list_set = [self.m0, self.m1, self.m2, self.m3, self.m4, self.m5]
        self.color_set = [self.m0_colors, self.m1_colors, self.m2_colors, self.m3_colors, self.m4_colors, self.m5_colors]
        self.label_set = [self.m0_labels, self.m1_labels, self.m2_labels, self.m3_labels, self.m4_labels, self.m5_labels]
        self.xlabel_set = [self.m0_xlabel, self.m1_xlabel, self.m2_xlabel, self.m3_xlabel, self.m4_xlabel, self.m5_xlabel]
        self.ylabel_set = [self.m0_ylabel, self.m1_ylabel, self.m2_ylabel, self.m3_ylabel, self.m4_ylabel, self.m5_ylabel]
        self.title_set = [self.m0_title, self.m1_title, self.m2_title, self.m3_title, self.m4_title, self.m5_title]
        self.fig, self.axs = plt.subplots(1, 6, figsize=(18,4.5))

        # Init axes
        self.line = []
        for data_idx, ax in enumerate(self.axs):
            for sub_data_idx in range(len(self.list_set[data_idx])):
                line, = ax.plot([], [], lw=0.8, color=self.color_set[data_idx][sub_data_idx], 
                                                label=self.label_set[data_idx][sub_data_idx])
                self.line.append(line)

            ax.set_xlabel(self.xlabel_set[data_idx])  # Add an x-label to the axes.
            ax.set_ylabel(self.ylabel_set[data_idx])  # Add a y-label to the axes.
            ax.set_title(self.title_set[data_idx])  # Add a title to the axes.            
            ax.grid()  # Add a legend.
            ax.legend()  # Add a legend.

    '''
    Callback functions
    ''' 
    def EgoLocalPoseCB(self, msg):
        self.height = msg.pose.position.z
        
    def TargetStatesCB(self, msg):
        # Append horizontal
        if not len(self.step_list) < BUF_SIZE:
            self.step_list.pop(0)
            self.m0[Val.MARKER_0.value].pop(0)
            self.m1[Val.MARKER_1.value].pop(0)
            self.m2[Val.MARKER_2.value].pop(0)
            self.m3[Val.MARKER_3.value].pop(0)
            self.m4[Val.MARKER_4.value].pop(0)
            self.m5[Val.MARKER_5.value].pop(0)

        # Append horizontal
        self.step_list.append(self.step)
        self.step += 1

        # Append vertical
        for id, aruco in enumerate(msg.aruco_states):
            if not aruco.is_detected:
                if id == 0:
                    self.m0[Val.MARKER_0.value].append(self.prev_freq_deg[id])
                elif id == 1:
                    self.m1[Val.MARKER_1.value].append(self.prev_freq_deg[id])
                elif id == 2:
                    self.m2[Val.MARKER_2.value].append(self.prev_freq_deg[id])
                elif id == 3:
                    self.m3[Val.MARKER_3.value].append(self.prev_freq_deg[id])
                elif id == 4:
                    self.m4[Val.MARKER_4.value].append(self.prev_freq_deg[id])
                elif id == 5:
                    self.m5[Val.MARKER_5.value].append(self.prev_freq_deg[id])

            else:
                if id == 0:
                    self.m0[Val.MARKER_0.value].append(aruco.frequency_degree)
                elif id == 1:
                    self.m1[Val.MARKER_1.value].append(aruco.frequency_degree)
                elif id == 2:
                    self.m2[Val.MARKER_2.value].append(aruco.frequency_degree)
                elif id == 3:
                    self.m3[Val.MARKER_3.value].append(aruco.frequency_degree)
                elif id == 4:
                    self.m4[Val.MARKER_4.value].append(aruco.frequency_degree)
                elif id == 5:
                    self.m5[Val.MARKER_5.value].append(aruco.frequency_degree)

                self.prev_freq_deg[id] = aruco.frequency_degree


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
            xlim = self.GetXLim(self.step_list)
            ax.set_ylim(ylim[0], ylim[1])
            ax.set_xlim(xlim[0], xlim[1])
            ax.axhline(20, xlim[0], xlim[1], color=('r'), linestyle='solid', linewidth=0.5)


            for sub_data_idx in range(len(self.list_set[data_idx])):
                self.line[line_idx].set_data(copy.deepcopy(self.step_list), copy.deepcopy(self.list_set[data_idx][sub_data_idx]))

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
            return [min(lst), min(lst)+10]
        else:
            return [min(lst), max(lst)]


if __name__ == "__main__":
    # Init node
    rospy.init_node('marker_pose_evaluation')
    plotting = Plotting()

    # Init subscriber
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)
    target_states_sub = rospy.Subscriber("/kuam/data/aruco_tracking/target_states", ArucoStates, plotting.TargetStatesCB)
    local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, plotting.EgoLocalPoseCB)

    # Animation
    ani = FuncAnimation(plotting.fig, plotting.UdatePlot, interval=1000)
    plt.show(block=True) 