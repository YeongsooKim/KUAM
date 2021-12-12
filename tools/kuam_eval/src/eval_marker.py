#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from utils import *
from enum import Enum
import copy
import numpy as mp

import tf2_geometry_msgs
import tf2_ros

# Message
from uav_msgs.msg import Chat
from kuam_msgs.msg import ArucoState
from kuam_msgs.msg import ArucoStates
from geometry_msgs.msg import Pose

MARGIN_RATIO = 0.3
BUF_SIZE = 256

# Data structure
class Val(Enum):
    POSITION_Z = 0
    FREQUENCY_Z = 0

# Init plotting class
class Plotting:
    def __init__(self):
        self.InitPlot()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.is_first = True
        self.prev_z = 0.0

    def InitPlot(self):
        # Horizontal axis
        self.step_list = []
        self.step = 0
        
        # Vertical axes
        self.z = [[]]
        self.z_colors = ['black']
        self.z_labels = ['position']
        self.z_xlabel = 'step []'
        self.z_ylabel = 'distance [m]'
        self.z_title = "Position Z"

        self.zdft = [[]]
        self.zdft_colors = ['black']
        self.zdft_labels = ['amplitude']
        self.zdft_xlabel = 'step []'
        self.zdft_ylabel = 'magnitude []'
        self.zdft_title = "Frequency Z"

        self.list_set = [self.z, self.zdft]
        self.color_set = [self.z_colors, self.zdft_colors]
        self.label_set = [self.z_labels, self.zdft_labels]
        self.xlabel_set = [self.z_xlabel, self.zdft_xlabel]
        self.ylabel_set = [self.z_ylabel, self.zdft_ylabel]
        self.title_set = [self.z_title, self.zdft_title]
        plt.rc('font', size=20)
        self.fig, self.axs = plt.subplots(1, 2, figsize=(7,4.5))

        # Init axes
        self.line = []
        for data_idx, ax in enumerate(self.axs):
            for sub_data_idx in range(len(self.list_set[data_idx])):
                line, = ax.plot([], [], lw=1, color=self.color_set[data_idx][sub_data_idx], 
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
    def TargetStatesCB(self, msg):
        for aruco in msg.aruco_states:
            if aruco.id != 2:
                continue
                
            if not aruco.is_detected:
                break

            if self.is_first:
                self.is_first = False
                self.prev_z = aruco.pose.position.z
            
            # Append horizontal
            if not len(self.step_list) < BUF_SIZE:
                self.step_list.pop(0)
                self.z[Val.POSITION_Z.value].pop(0)
                self.zdft[Val.FREQUENCY_Z.value].pop(0)

            self.step_list.append(self.step)
            self.step += 1

            # Append vertical
            # self.z[Val.POSITION_Z.value].append(abs(aruco.pose.position.z - self.prev_z))
            self.z[Val.POSITION_Z.value].append(abs(aruco.pose.position.z))
            self.zdft[Val.FREQUENCY_Z.value] = self.DFT(self.z[Val.POSITION_Z.value])
            self.prev_z = aruco.pose.position.z

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
    
            if data_idx == 0:
                ax.set_ylim(self.GetYLim(self.list_set[data_idx]))
                ax.set_xlim(self.GetXLim(self.step_list))
            else:
                ax.set_ylim(0, 30)
                ax.set_xlim(0, BUF_SIZE)

            for sub_data_idx in range(len(self.list_set[data_idx])):
                if data_idx == 0:
                    self.line[line_idx].set_data(copy.deepcopy(self.step_list), copy.deepcopy(self.list_set[data_idx][sub_data_idx]))
                else:
                    x_idx = list(range(len(self.list_set[data_idx][sub_data_idx])))
                    self.line[line_idx].set_data(x_idx, copy.deepcopy(self.list_set[data_idx][sub_data_idx]))

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

    def DFT(self, time_domain_values):
        N = len(time_domain_values)
        K = N

        output = []

        for k in range(K):
            sum = complex(0., 0.)
            for n in range(N):
                real = 10*time_domain_values[n] * cos(((2*pi)/N) * k * n)
                imag = 10*time_domain_values[n] * sin(((2*pi)/N) * k * n)
                w = complex(real, -imag)
                sum += w

            output.append(abs(sum))

        return output
    
if __name__ == "__main__":
    # Init node
    rospy.init_node('marker_pose_evaluation')
    plotting = Plotting()

    # Init subscriber
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)
    target_states_sub = rospy.Subscriber("/kuam/data/aruco_tracking/target_states", ArucoStates, plotting.TargetStatesCB)

    # Animation
    ani = FuncAnimation(plotting.fig, plotting.UdatePlot, interval=500)
    plt.show(block=True) 