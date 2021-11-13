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
    LANDING_POINT_X_DIFF = 0
    LANDING_POINT_Y = 0
    LANDING_POINT_Y_DIFF = 0
    LANDING_POINT_Z = 0
    LANDING_POINT_Z_DIFF = 0
    EGO_VEHICLE_HEIGHT = 0
    DUMMY = 0

# Init plotting class
class Plotting:
    def __init__(self):
        self.InitPlot()
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.setpoint = Setpoint()
        self.is_init_threshold_point = [False, False, False]
        self.threshold_time = [0.0, 0.0, 0.0]
        self.height = 0.0
        self.prev_time = rospy.Time.now()

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0

    def InitPlot(self):
        # Horizontal axis
        self.init_time = None
        self.is_first = True
        self.time_list = []
        
        # Vertical axes
        self.x = [[]]
        self.x_colors = ['black']
        self.x_labels = ['landing point']
        self.x_xlabel = 'time [s]'
        self.x_ylabel = 'distance [m]'
        self.x_title = "Landing Point X"

        self.y = [[]]
        self.y_colors = ['black']
        self.y_labels = ['landing point']
        self.y_xlabel = 'time [s]'
        self.y_ylabel = 'distance [m]'
        self.y_title = "Landing Point Y"

        self.z = [[]]
        self.z_colors = ['black']
        self.z_labels = ['landing point']
        self.z_xlabel = 'time [s]'
        self.z_ylabel = 'distance [m]'
        self.z_title = "Landing Point Z"

        self.h = [[]]
        self.h_colors = ['red']
        self.h_labels = ['height']
        self.h_xlabel = 'time [s]'
        self.h_ylabel = 'distance [m]'
        self.h_title = "Ego Vehicle Height"

        self.dx = [[]]
        self.dx_colors = ['black']
        self.dx_labels = ['difference']
        self.dx_xlabel = 'time [s]'
        self.dx_ylabel = 'velocity [m/s]'
        self.dx_title = "Landing Point X Diff"

        self.dy = [[]]
        self.dy_colors = ['black']
        self.dy_labels = ['difference']
        self.dy_xlabel = 'time [s]'
        self.dy_ylabel = 'velocity [m/s]'
        self.dy_title = "Landing Point Y Diff"

        self.dz = [[]]
        self.dz_colors = ['black']
        self.dz_labels = ['difference']
        self.dz_xlabel = 'time [s]'
        self.dz_ylabel = 'velocity [m/s]'
        self.dz_title = "Landing Point Z Diff"

        self.d = [[]]
        self.d_colors = ['red']
        self.d_labels = ['']
        self.d_xlabel = ''
        self.d_ylabel = ''
        self.d_title = "Dummy"

        self.list_set = [self.x, self.y, self.z, self.h, self.dx, self.dy, self.dz, self.d]
        self.color_set = [self.x_colors, self.y_colors, self.z_colors, self.h_colors, self.dx_colors, self.dy_colors, self.dz_colors, self.d_colors]
        self.label_set = [self.x_labels, self.y_labels, self.z_labels, self.h_labels, self.dx_labels, self.dy_labels, self.dz_labels, self.d_labels]
        self.xlabel_set = [self.x_xlabel, self.y_xlabel, self.z_xlabel, self.h_xlabel, self.dx_xlabel, self.dy_xlabel, self.dz_xlabel, self.d_xlabel]
        self.ylabel_set = [self.x_ylabel, self.y_ylabel, self.z_ylabel, self.h_ylabel, self.dx_ylabel, self.dy_ylabel, self.dz_ylabel, self.d_ylabel]
        self.title_set = [self.x_title, self.y_title, self.z_title, self.h_title, self.dx_title, self.dy_title, self.dz_title, self.d_title]
        self.fig, self.axs = plt.subplots(2, 4, figsize=(15,9))

        # Init axes
        self.line = []
        for raw_idx, raw_ax in enumerate(self.axs):
            for col_idx, col_ax in enumerate(raw_ax):
                data_idx = raw_idx*len(raw_ax) + col_idx

                for sub_data_idx in range(len(self.list_set[data_idx])):
                    line, = col_ax.plot([], [], lw=2, color=self.color_set[data_idx][sub_data_idx], 
                                                    label=self.label_set[data_idx][sub_data_idx])
                    self.line.append(line)

                col_ax.set_xlabel(self.xlabel_set[data_idx])  # Add an x-label to the axes.
                col_ax.set_ylabel(self.ylabel_set[data_idx])  # Add a y-label to the axes.
                col_ax.set_title(self.title_set[data_idx])  # Add a title to the axes.            
                col_ax.grid()
                col_ax.legend()  # Add a legend.

    '''
    Callback functions
    ''' 
    def ProcessCB(self, timer):
        if not self.setpoint.landing_state.is_vehicle_detected and not self.setpoint.landing_state.is_marker_detected:
            return

        # Append horizontal
        if self.is_first:
            self.is_first = False
            self.init_time = rospy.Time.now()
            self.prev_x = self.setpoint.target_pose.position.x
            self.prev_y = self.setpoint.target_pose.position.y
            self.prev_z = self.setpoint.target_pose.position.z


        if len(self.time_list) == 0:
            prev_time = 0
        else:
            prev_time = self.time_list[-1]

        elapse = (rospy.Time.now() - self.init_time).to_sec()
        cur_time = elapse
        self.time_list.append(elapse)

        # Append raw 0 vertical
        self.x[Val.LANDING_POINT_X.value].append(self.setpoint.target_pose.position.x)
        self.y[Val.LANDING_POINT_Y.value].append(self.setpoint.target_pose.position.y)
        self.z[Val.LANDING_POINT_Z.value].append(self.setpoint.target_pose.position.z)
        self.h[Val.EGO_VEHICLE_HEIGHT.value].append(self.height)

        # Append raw 1 vertical
        dt = cur_time - prev_time
        if dt < 0.001:
            dt = 0.1

        dif_x = (self.prev_x - self.setpoint.target_pose.position.x)/dt
        dif_y = (self.prev_y - self.setpoint.target_pose.position.y)/dt
        dif_z = (self.prev_z - self.setpoint.target_pose.position.z)/dt
        self.dx[Val.LANDING_POINT_X_DIFF.value].append(dif_x)
        self.dy[Val.LANDING_POINT_Y_DIFF.value].append(dif_y)
        self.dz[Val.LANDING_POINT_Z_DIFF.value].append(dif_z)
        self.d[Val.DUMMY.value].append(0.0)

        self.prev_x = self.setpoint.target_pose.position.x
        self.prev_y = self.setpoint.target_pose.position.y
        self.prev_z = self.setpoint.target_pose.position.z


    def EgoLocalPoseCB(self, msg):
        self.height = msg.pose.position.z

    def FittingPlaneCB(self, msg):
        if msg.big_is_valid and msg.big_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[0]:
            rospy.logwarn("big init")
            self.is_init_threshold_point[0] = True
            self.threshold_time[0] = (rospy.Time.now() - self.init_time).to_sec()

        if msg.medium_is_valid and msg.medium_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[1]:
            rospy.logwarn("medium init")
            self.is_init_threshold_point[1] = True
            self.threshold_time[1] = (rospy.Time.now() - self.init_time).to_sec()

        if msg.small_is_valid and msg.small_z_to_normal_deg <= 28.0 and not self.is_init_threshold_point[2]:
            rospy.logwarn("small init")
            self.is_init_threshold_point[2] = True
            self.threshold_time[2] = (rospy.Time.now() - self.init_time).to_sec()

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
        data_idx = 0
        line_idx = 0
        for raw_ax in self.axs:
            for col_ax in raw_ax:

                ylim = self.GetYLim(self.list_set[data_idx])
                col_ax.set_ylim(ylim[0], ylim[1])
                col_ax.set_xlim(self.GetXLim(self.time_list))

                if self.is_init_threshold_point[2]:
                    col_ax.vlines((self.threshold_time[0], self.threshold_time[1], self.threshold_time[2]), 
                                ylim[0], ylim[1], color=('r', 'g', 'b'), linestyle='solid', linewidth=0.5)

                elif self.is_init_threshold_point[1]:
                    col_ax.vlines((self.threshold_time[0], self.threshold_time[1]), 
                                ylim[0], ylim[1], color=('r', 'g'), linestyle='solid', linewidth=0.5)

                elif self.is_init_threshold_point[0]:
                    col_ax.vlines(self.threshold_time[0], ylim[0], ylim[1], color='r', linestyle='solid', linewidth=0.5)

                for sub_data_idx in range(len(self.list_set[data_idx])):
                    
                    if len(self.time_list) == len(self.list_set[data_idx][sub_data_idx]):
                        self.line[line_idx].set_data(self.time_list, self.list_set[data_idx][sub_data_idx])
                    line_idx += 1
                
                data_idx += 1

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


    def GetXLim(self, time_list):
        if len(time_list) == 0:
            return [0, 10]
        elif time_list[-1] < 10.0:
            return [0, 10]
        else:
            return [0, time_list[-1]]

    
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