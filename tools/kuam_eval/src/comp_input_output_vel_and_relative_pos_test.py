#!/usr/bin/env python3
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from utils import *
from enum import Enum

from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from kuam_msgs.msg import Setpoint
from uav_msgs.msg import Chat

MARGIN_RATIO = 0.3
BUF_SIZE = 200

class Val(Enum):
    OUTPUT_POS_X = 0
    OUTPUT_POS_Y = 0
    OUTPUT_POS_Z = 0
    OUTPUT_YAW = 0

is_detected_ = False
is_start_ = False
target_pos_ = Point()
target_yaw_ = 0.0

class Plotting:
    def __init__(self):
        self.init_time = None
        self.elapsed_time = 0.0

        self.time_s = []
        
        self.x = [[]]
        self.x_colors = ['red']
        self.x_labels = ['x output pos']
        self.x_xlabel = 'time [s]'
        self.x_ylabel = 'distance [m]'
        self.x_title = "'x' Position"

        self.y = [[]]
        self.y_colors = ['green']
        self.y_labels = ['y output pos']
        self.y_xlabel = 'time [s]'
        self.y_ylabel = 'distance [m]'
        self.y_title = "'y' Position"

        self.z = [[]]
        self.z_colors = ['blue']
        self.z_labels = ['z output pos']
        self.z_xlabel = 'time [s]'
        self.z_ylabel = 'distance [m]'
        self.z_title = "'z' Position"

        self.yaw = [[]]
        self.yaw_colors = ['purple']
        self.yaw_labels = ['output yaw']
        self.yaw_xlabel = 'time [s]'
        self.yaw_ylabel = 'angle [deg]'
        self.yaw_title = "'yaw' Angle"

        self.list_set = [self.x, self.y, self.z, self.yaw]
        self.color_set = [self.x_colors, self.y_colors, self.z_colors, self.yaw_colors]
        self.label_set = [self.x_labels, self.y_labels, self.z_labels, self.yaw_labels]
        self.xlabel_set = [self.x_xlabel, self.y_xlabel, self.z_xlabel, self.yaw_xlabel]
        self.ylabel_set = [self.x_ylabel, self.y_ylabel, self.z_ylabel, self.yaw_ylabel]
        self.title_set = [self.x_title, self.y_title, self.z_title, self.yaw_title]
        self.fig, self.axs = plt.subplots(1, 4, num=1)

    '''
    Callback functions
    ''' 
    def ProcessCB(self, timer):
        if not is_start_:
            self.init_time = rospy.Time.now()
            return

        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        self.time_s.append(elapse.to_sec())

        if is_detected_:
            global target_pos_

            self.x[Val.OUTPUT_POS_X.value].append(target_pos_.x)
            self.y[Val.OUTPUT_POS_Y.value].append(target_pos_.y)
            self.z[Val.OUTPUT_POS_Z.value].append(target_pos_.z)
            self.yaw[Val.OUTPUT_YAW.value].append(target_yaw_)

        else:
            self.x[Val.OUTPUT_POS_X.value].append(0.0)
            self.y[Val.OUTPUT_POS_Y.value].append(0.0)
            self.z[Val.OUTPUT_POS_Z.value].append(0.0)
            self.yaw[Val.OUTPUT_YAW.value].append(0.0)

    def SetpointCB(self, msg):
        global target_pos_
        target_pos_ = msg.target_pose.position

        global target_yaw_
        target_yaw_ = GetYawDeg(msg.target_pose.orientation)

        global is_detected_
        if msg.landing_state.is_marker_detected or msg.landing_state.is_vehicle_detected:
            is_detected_ = True

        global state_machine_cmd_id_
        if msg.landing_state.mode == "standby":
            state_machine_cmd_id_ = 0
        else:
            if msg.landing_state.mode == "mode1":
                state_machine_cmd_id_ = 1
            else:
                state_machine_cmd_id_ = 2

    def ChatCB(self, msg):
        cmd = msg.msg
        if cmd == 'pause':
            ani.event_source.stop()
            rospy.loginfo("[eval] pause plotting")
        elif cmd == 'play':
            ani.event_source.start()
            rospy.loginfo("[eval] play plotting")
        elif cmd == 'start':
            global is_start_
            is_start_ = True
 
    '''
    Plot
    '''
    def PlotInit(self):
        pass

    def UdatePlot(self, frame):
        global is_start_
        if not is_start_:
            return

        if 

        type = 0
        
        for i, ax in enumerate(self.axs):
            ax.clear()
            ylim = self.GetLimMinMax(self.list_set[0])
            ax.set_ylim(ylim[0], ylim[1])
            ax.plot(self.time_s, self.list_set[0][i])

            ax.set_xlabel(self.xlabel_set[i])  # Add an x-label to the axes.
            ax.set_ylabel(self.ylabel_set[i])  # Add a y-label to the axes.
            ax.set_title(self.title_set[i])  # Add a title to the axes.            
            ax.legend()  # Add a legend.


        # for i in range(len(self.axs)):
        #     for j in range(len(self.axs[i])):
        #         self.axs[i][j].clear()
        #         ylim = self.GetLimMinMax(self.list_set[type])
        #         self.axs[i][j].set_ylim(ylim[0], ylim[1])
                
        #         for element in range(len(self.list_set[type])):
        #             if (len(self.time_s) == len(self.list_set[type][element])):
        #                 try:
        #                     self.axs[i][j].plot(self.time_s, self.list_set[type][element], color=self.color_set[type][element], label=self.label_set[type][element])
        #                 except:
        #                     pass

        #         self.axs[i][j].set_xlabel(self.xlabel_set[type])  # Add an x-label to the axes.
        #         self.axs[i][j].set_ylabel(self.ylabel_set[type])  # Add a y-label to the axes.
        #         self.axs[i][j].set_title(self.title_set[type])  # Add a title to the axes.            
        #         self.axs[i][j].legend()  # Add a legend.

        #         type += 1

        self.fig.tight_layout()

    '''
    Util functions
    '''
    def GetLimMinMax(self, lists):
        ylim = [0, BUF_SIZE - 1]
        lengths = []
        for lst in lists:
            lengths.append(len(lst))

        # If empty list is exist, return
        for leng in lengths:
            if leng == 0:
                return ylim

        list_max = -sys.maxsize - 1
        for lst in lists:
            if max(lst) > list_max:
                list_max = max(lst)

        list_min = sys.maxsize
        for lst in lists:
            if min(lst) < list_min:
                list_min = min(lst)
        
        if abs(list_max - list_min) < 0.1:
            return [-0.5, 0.5]

        margin = (list_max - list_min)*MARGIN_RATIO/2.0
        ylim = [float(list_min) - margin, float(list_max) + margin]

        return ylim

if __name__ == "__main__":

    rospy.init_node('comp_input_output_vel_and_relative_pos')
    plotting = Plotting()

    setpoint_sub = rospy.Subscriber('/kuam/maneuver/state_machine/setpoint', Setpoint, plotting.SetpointCB)
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

    freq = 10.0
    process_timer = rospy.Timer(rospy.Duration(1.0/freq), plotting.ProcessCB)

    ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
    plt.show(block=True) 