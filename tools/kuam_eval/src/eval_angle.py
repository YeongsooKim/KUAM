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
    MEASURED_ANGLE = 0
    EGO_HEIGHT = 0

# Init plotting class
class Plotting:
    def __init__(self):
        self.InitPlot()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def InitPlot(self):
        # Horizontal axis
        self.step_list = []
        self.step = 0
        
        # Vertical axes
        self.a = [[]]
        self.a_colors = ['black', ]
        self.a_labels = ['angle', ]
        self.a_xlabel = 'step []'
        self.a_ylabel = 'degree [deg]'
        self.a_title = "Angle with Fitting Plane"

        self.h = [[]]
        self.h_colors = ['red']
        self.h_labels = ['height']
        self.h_xlabel = 'step []'
        self.h_ylabel = 'distance [m]'
        self.h_title = "Ego Vehicle Height"

        self.list_set = [self.a, self.h]
        self.color_set = [self.a_colors, self.h_colors]
        self.label_set = [self.a_labels, self.h_labels]
        self.xlabel_set = [self.a_xlabel, self.h_xlabel]
        self.ylabel_set = [self.a_ylabel, self.h_ylabel]
        self.title_set = [self.a_title, self.h_title]
        self.fig, self.axs = plt.subplots(1, 2, figsize=(10,6))

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
            ax.legend()  # Add a legend.

    '''
    Callback functions
    ''' 
    def EgoLocalPoseCB(self, msg):
        self.height = msg.pose.position.z
        
    def FittingPlaneCB(self, msg):
        # Append horizontal
        self.step_list.append(self.step)
        self.step += 1

        # Append vertical
        self.a[Val.MEASURED_ANGLE.value].append(msg.big_z_to_normal_deg)
        self.h[Val.EGO_HEIGHT.value].append(self.height)

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
    
            ax.set_ylim(self.GetYLim(self.list_set[data_idx]))
            ax.set_xlim(self.GetXLim(self.step_list))

            for sub_data_idx in range(len(self.list_set[data_idx])):
                if len(self.step_list) == len(self.list_set[data_idx][sub_data_idx]):
                    self.line[line_idx].set_data(self.step_list, self.list_set[data_idx][sub_data_idx])
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


    def GetXLim(self, step_list):
        if len(step_list) == 0:
            return [0, 10]
        elif step_list[-1] < 10.0:
            return [0, 10]
        else:
            return [0, step_list[-1]]

    
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