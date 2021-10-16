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
from kuam_msgs.msg import Setpoint
from kuam_msgs.msg import FittingPlane
from geometry_msgs.msg import PoseStamped

MARGIN_RATIO = 0.3
BUF_SIZE = 200

# Data structure
class Val(Enum):
    LANDING_POINT_X = 0
    LANDING_POINT_Y = 0
    LANDING_POINT_Z = 0
    EGO_VEHICLE_HEIGHT = 0

# Init plotting class
class Plotting:
    def __init__(self):
        self.InitPlot()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.is_init_threshold_point = [False, False, False]
        self.threshold_step = [0.0, 0.0, 0.0]

        self.height = 0.0
        self.setpoint = Setpoint()

    def InitPlot(self):
        # Horizontal axis
        self.step_list = []
        self.step = 0
        
        # Vertical axes
        self.x = [[]]
        self.x_colors = ['black']
        self.x_labels = ['landing point']
        self.x_xlabel = 'step []'
        self.x_ylabel = 'distance [m]'
        self.x_title = "Landing Point X"

        self.y = [[]]
        self.y_colors = ['black']
        self.y_labels = ['landing point']
        self.y_xlabel = 'step []'
        self.y_ylabel = 'distance [m]'
        self.y_title = "Landing Point Y"

        self.z = [[]]
        self.z_colors = ['black']
        self.z_labels = ['landing point']
        self.z_xlabel = 'step []'
        self.z_ylabel = 'distance [m]'
        self.z_title = "Landing Point Z"

        self.h = [[]]
        self.h_colors = ['red']
        self.h_labels = ['height']
        self.h_xlabel = 'step []'
        self.h_ylabel = 'distance [m]'
        self.h_title = "Ego Vehicle Height"

        self.list_set = [self.x, self.y, self.z, self.h]
        self.color_set = [self.x_colors, self.y_colors, self.z_colors, self.h_colors]
        self.label_set = [self.x_labels, self.y_labels, self.z_labels, self.h_labels]
        self.xlabel_set = [self.x_xlabel, self.y_xlabel, self.z_xlabel, self.h_xlabel]
        self.ylabel_set = [self.x_ylabel, self.y_ylabel, self.z_ylabel, self.h_ylabel]
        self.title_set = [self.x_title, self.y_title, self.z_title, self.h_title]
        self.fig, self.axs = plt.subplots(1, 4, figsize=(18,6))

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
        if not self.setpoint.landing_state.is_vehicle_detected and not self.setpoint.landing_state.is_marker_detected:
            return

        # Append horizontal
        self.step_list.append(self.step)
        self.step += 1

        # Append vertical
        self.x[Val.LANDING_POINT_X.value].append(self.setpoint.target_pose.position.x)
        self.y[Val.LANDING_POINT_Y.value].append(self.setpoint.target_pose.position.y)
        self.z[Val.LANDING_POINT_Z.value].append(self.setpoint.target_pose.position.z)
        self.h[Val.EGO_VEHICLE_HEIGHT.value].append(self.height)

    def EgoLocalPoseCB(self, msg):
        self.height = msg.pose.position.z

    def FittingPlaneCB(self, msg):
        if msg.big_is_valid and msg.big_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[0]:
            rospy.logwarn("big init")
            self.is_init_threshold_point[0] = True
            self.threshold_step[0] = self.step - 1

        if msg.medium_is_valid and msg.medium_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[1]:
            rospy.logwarn("medium init")
            self.is_init_threshold_point[1] = True
            self.threshold_step[1] = self.step - 1

        if msg.small_is_valid and msg.small_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[2]:
            rospy.logwarn("small init")
            self.is_init_threshold_point[2] = True
            self.threshold_step[2] = self.step - 1

    def SetpointCB(self, msg):
        self.setpoint = msg

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
            ax.set_ylim(ylim[0], ylim[1])
            ax.set_xlim(self.GetXLim(self.step_list))

            if self.is_init_threshold_point[0]:
                ax.vlines(self.threshold_step[0], ylim[0], ylim[1], color='red', linestyle='solid', linewidth=0.5)

            if self.is_init_threshold_point[1]:
                ax.vlines(self.threshold_step[1], ylim[0], ylim[1], color='green', linestyle='solid', linewidth=0.5)

            if self.is_init_threshold_point[2]:
                ax.vlines(self.threshold_step[2], ylim[0], ylim[1], color='blue', linestyle='solid', linewidth=0.5)

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
    local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, plotting.EgoLocalPoseCB)
    setpoint_sub = rospy.Subscriber("/kuam/maneuver/state_machine/setpoint", Setpoint, plotting.SetpointCB)
    fitting_plane_sub = rospy.Subscriber("/kuam/data/aruco_tracking/fitting_plane", FittingPlane, plotting.FittingPlaneCB)

    # Init timer
    freq = 10.0
    process_timer = rospy.Timer(rospy.Duration(1.0/freq), plotting.ProcessCB)

    # Animation
    ani = FuncAnimation(plotting.fig, plotting.UdatePlot)
    plt.show(block=True) 