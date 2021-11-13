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
    INPUT_VEL_X = 0
    OUTPUT_VEL_X = 1
    INPUT_VEL_Y = 0
    OUTPUT_VEL_Y = 1
    INPUT_VEL_Z = 0
    OUTPUT_VEL_Z = 1
    INPUT_YAW_RATE = 0
    OUTPUT_YAW_RATE = 1

is_detected_ = False
is_start_ = False
input_vel_ = Twist()
output_vel_ = Twist()
input_yaw_rate_ = 0.0
output_yaw_rate_ = 0.0
target_pos_ = Point()
target_yaw_ = 0.0

class Plotting:
    def __init__(self):
        self.init_time = None
        self.elapsed_time = 0.0

        self.time = []
        
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

        self.vx = [[], []]
        self.vx_colors = ['black', 'red']
        self.vx_labels = ['input vel [m/s]', 'output vel [m/s]']
        self.vx_xlabel = 'time [s]'
        self.vx_ylabel = 'velocity [m/s]'
        self.vx_title = "Input and Output X Velocity"

        self.vy = [[], []]
        self.vy_colors = ['black', 'green']
        self.vy_labels = ['input vel [m/s]', 'output vel [m/s]']
        self.vy_xlabel = 'time [s]'
        self.vy_ylabel = 'velocity [m/s]'
        self.vy_title = "Input and Output Y Velocity"

        self.vz = [[], []]
        self.vz_colors = ['black', 'blue']
        self.vz_labels = ['input vel [m/s]', 'output vel [m/s]']
        self.vz_xlabel = 'time [s]'
        self.vz_ylabel = 'velocity [m/s]'
        self.vz_title = "Input and Output Z Velocity"

        self.vyaw = [[], []]
        self.vyaw_colors = ['black', 'purple']
        self.vyaw_labels = ['input yaw rate [deg/s]', 'output yaw rate [deg/s]']
        self.vyaw_xlabel = 'time [s]'
        self.vyaw_ylabel = 'angular velocity [deg/s]'
        self.vyaw_title = 'Input and Output Yaw Rate'

        self.list_set = [self.x, self.y, self.z, self.yaw, self.vx, self.vy, self.vz, self.vyaw]
        self.color_set = [self.x_colors, self.y_colors, self.z_colors, self.yaw_colors, self.vx_colors, self.vy_colors, self.vz_colors, self.vyaw_colors]
        self.label_set = [self.x_labels, self.y_labels, self.z_labels, self.yaw_labels, self.vx_labels, self.vy_labels, self.vz_labels, self.vyaw_labels]
        self.xlabel_set = [self.x_xlabel, self.y_xlabel, self.z_xlabel, self.yaw_xlabel, self.vx_xlabel, self.vy_xlabel, self.vz_xlabel, self.vyaw_xlabel]
        self.ylabel_set = [self.x_ylabel, self.y_ylabel, self.z_ylabel, self.yaw_ylabel, self.vx_ylabel, self.vy_ylabel, self.vz_ylabel, self.vyaw_ylabel]
        self.title_set = [self.x_title, self.y_title, self.z_title, self.yaw_title, self.vx_title, self.vy_title, self.vz_title, self.vyaw_title]
        self.fig, self.axs = plt.subplots(2, 4)

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
                col_ax.legend()  # Add a legend.

    '''
    Callback functions
    ''' 
    def ProcessCB(self, timer):
        if not is_start_:
            self.init_time = rospy.Time.now()
            return

        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        self.time.append(elapse.to_sec())

        if is_detected_:
            global input_vel_
            global output_vel_
            global target_pos_

            self.x[Val.OUTPUT_POS_X.value].append(target_pos_.x)
            self.y[Val.OUTPUT_POS_Y.value].append(target_pos_.y)
            self.z[Val.OUTPUT_POS_Z.value].append(target_pos_.z)
            self.yaw[Val.OUTPUT_YAW.value].append(target_yaw_)

            self.vx[Val.INPUT_VEL_X.value].append(input_vel_.linear.x)
            self.vx[Val.OUTPUT_VEL_X.value].append(output_vel_.linear.x)
            self.vy[Val.INPUT_VEL_Y.value].append(input_vel_.linear.y)
            self.vy[Val.OUTPUT_VEL_Y.value].append(output_vel_.linear.y)
            self.vz[Val.INPUT_VEL_Z.value].append(input_vel_.linear.z)
            self.vz[Val.OUTPUT_VEL_Z.value].append(output_vel_.linear.z)
            self.vyaw[Val.INPUT_YAW_RATE.value].append(input_yaw_rate_)
            self.vyaw[Val.OUTPUT_YAW_RATE.value].append(output_yaw_rate_)

        else:
            self.x[Val.OUTPUT_POS_X.value].append(0.0)
            self.y[Val.OUTPUT_POS_Y.value].append(0.0)
            self.z[Val.OUTPUT_POS_Z.value].append(0.0)
            self.yaw[Val.OUTPUT_YAW.value].append(0.0)

            self.vx[Val.INPUT_VEL_X.value].append(0.0)
            self.vx[Val.OUTPUT_VEL_X.value].append(0.0)
            self.vy[Val.INPUT_VEL_Y.value].append(0.0)
            self.vy[Val.OUTPUT_VEL_Y.value].append(0.0)
            self.vz[Val.INPUT_VEL_Z.value].append(0.0)
            self.vz[Val.OUTPUT_VEL_Z.value].append(0.0)
            self.vyaw[Val.INPUT_YAW_RATE.value].append(0.0)
            self.vyaw[Val.OUTPUT_YAW_RATE.value].append(0.0)

    def SetpointCB(self, msg):
        global input_vel_
        input_vel_ = msg.vel

        global target_pos_
        target_pos_ = msg.target_pose.position

        global input_yaw_rate_
        input_yaw_rate_ = GetYawDeg(msg.yaw_rate.orientation)

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

    def VelocityCB(self, msg):
        global output_vel_
        output_vel_ = msg.twist

        global output_yaw_rate_
        output_yaw_rate_ = Rad2Deg(msg.twist.angular.z)

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
    def UdatePlot(self, frame):
        for raw_idx, raw_ax in enumerate(self.axs):
            for col_idx, col_ax in enumerate(raw_ax):
                data_idx = raw_idx*len(raw_ax) + col_idx

                col_ax.set_ylim(self.GetYLim(self.list_set[data_idx]))
                col_ax.set_xlim(self.GetXLim(self.time))

                for sub_data_idx in range(len(self.list_set[data_idx])):
                    line_idx = raw_idx*len(raw_ax) + col_idx*len(self.list_set[data_idx]) + sub_data_idx

                    self.line[line_idx].set_data(self.time, self.list_set[data_idx][sub_data_idx])

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

    def GetXLim(self, time):
        if len(time) == 0:
            return [0, 10]
        elif time[-1] < 10.0:
            return [0, 10]
        else:
            return [0, self.time[-1]]


if __name__ == "__main__":

    rospy.init_node('comp_input_output_vel_and_relative_pos')
    plotting = Plotting()

    setpoint_sub = rospy.Subscriber('/kuam/maneuver/state_machine/setpoint', Setpoint, plotting.SetpointCB)
    vel_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, plotting.VelocityCB)
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

    freq = 10.0
    process_timer = rospy.Timer(rospy.Duration(1.0/freq), plotting.ProcessCB)

    ani = FuncAnimation(plotting.fig, plotting.UdatePlot)
    plt.show(block=True) 