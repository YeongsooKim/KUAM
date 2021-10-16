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
# from message_msgs.msg import Message

MARGIN_RATIO = 0.3
BUF_SIZE = 200

# Data structure
class Val(Enum):
    GROUND_TRUTH_X = 0
    ESTIMATED_X = 1
    GROUND_TRUTH_Y = 0
    ESTIMATED_Y = 1
    GROUND_DIST = 0
    ESTIMATED_DIST = 1

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
        self.x = [[], []]
        self.x_colors = ['black', 'red']
        self.x_labels = ['ground truth x', 'estimated x']
        self.x_xlabel = 'step []'
        self.x_ylabel = 'distance [m]'
        self.x_title = "Title"

        self.y = [[], []]
        self.y_colors = ['black', 'green']
        self.y_labels = ['ground truth y', 'estimated y']
        self.y_xlabel = 'step []'
        self.y_ylabel = 'distance [m]'
        self.y_title = "Title"

        self.list_set = [self.x, self.y]
        self.color_set = [self.x_colors, self.y_colors]
        self.label_set = [self.x_labels, self.y_labels]
        self.xlabel_set = [self.x_xlabel, self.y_xlabel]
        self.ylabel_set = [self.x_ylabel, self.y_ylabel]
        self.title_set = [self.x_title, self.y_title]
        self.fig, self.axs = plt.subplots(1, 2, figsize=(15,9))

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
    def ProcessCB(self, timer):
        # Append horizontal
        self.step_list.append(self.step)
        self.step += 1

        # Append vertical
        self.x[Val.GROUND_TRUTH_X.value].append(0.0)
        self.x[Val.ESTIMATED_X.value].append(0.0)
        self.y[Val.GROUND_TRUTH_Y.value].append(0.0)
        self.y[Val.ESTIMATED_Y.value].append(0.0)


    def FuncCB(self, msg):
        # Todo
        pass


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
    # subscriber_sub = rospy.Subscriber("topic", Message, plotting.FuncCB)

    # Init timer
    # freq = 10.0
    # process_timer = rospy.Timer(rospy.Duration(1.0/freq), plotting.ProcessCB)

    # Animation
    ani = FuncAnimation(plotting.fig, plotting.UdatePlot)
    plt.show(block=True) 