#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import tf2_ros
import tf2_geometry_msgs
import sys
from utils import *
from enum import Enum

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from kuam_msgs.msg import ArucoState
from kuam_msgs.msg import Setpoint
from uav_msgs.msg import Chat

Y_AXIS_MARGIN = [0.5, 0.5, 0.5, 0.05, 0.05, 0.05]
Y_AXIS_MARGIN2 = [0.5, 0.5]
BUF_SIZE = 200

class Val(Enum):
    TARGET = 0
    CMD = 0
    OUTPUT_POS_X = 0
    OUTPUT_POS_Y = 0
    OUTPUT_POS_Z = 0
    INPUT_VEL_X = 0
    OUTPUT_VEL_X = 1
    INPUT_VEL_Y = 0
    OUTPUT_VEL_Y = 1
    INPUT_VEL_Z = 0
    OUTPUT_VEL_Z = 1

tfBuffer_ = None
listener_ = None
target_id_ = 0
state_machine_cmd_id_ = 0 # 0: not in cmd / 1: kuam ctrl / 2: px4 ctrl
input_vel_ = Twist()
output_vel_ = Twist()
target_pos_ = Point()

class Plotting:
    def __init__(self):
        self.init_time = None
        self.is_first = True
        self.elapsed_time = 0.0
        self.x_raw = []
        self.y_raw = []
        self.z_raw = []

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

        self.vx = [[], []]
        self.vx_colors = ['black', 'red']
        self.vx_labels = ['input vel [m/s]', 'output vel [m/s]']
        self.vx_xlabel = 'time [s]'
        self.vx_ylabel = 'velocity [m/s]'
        self.vx_title = "'x' Input and Output Velocity"

        self.vy = [[], []]
        self.vy_colors = ['black', 'green']
        self.vy_labels = ['input vel [m/s]', 'output vel [m/s]']
        self.vy_xlabel = 'time [s]'
        self.vy_ylabel = 'velocity [m/s]'
        self.vy_title = "'y' Input and Output Velocity"

        self.vz = [[], []]
        self.vz_colors = ['black', 'blue']
        self.vz_labels = ['input vel [m/s]', 'output vel [m/s]']
        self.vz_xlabel = 'time [s]'
        self.vz_ylabel = 'velocity [m/s]'
        self.vz_title = "'z' Input and Output Velocity"

        self.list_set = [self.x, self.y, self.z, self.vx, self.vy, self.vz]
        self.color_set = [self.x_colors, self.y_colors, self.z_colors, self.vx_colors, self.vy_colors, self.vz_colors]
        self.label_set = [self.x_labels, self.y_labels, self.z_labels, self.vx_labels, self.vy_labels, self.vz_labels]
        self.xlabel_set = [self.x_xlabel, self.y_xlabel, self.z_xlabel, self.vx_xlabel, self.vy_xlabel, self.vz_xlabel]
        self.ylabel_set = [self.x_ylabel, self.y_ylabel, self.z_ylabel, self.vx_ylabel, self.vy_ylabel, self.vz_ylabel]
        self.title_set = [self.x_title, self.y_title, self.z_title, self.vx_title, self.vy_title, self.vz_title]
        self.fig, self.axs = plt.subplots(2, 3, num=1)
         
        self.target = [[]]
        self.t_colors = ['red']
        self.t_labels = ['target']
        self.t_xlabel = 'time [s]'
        self.t_ylabel = 'marker id + 1 [ ]'
        self.t_title = 'Target Marker ID'

        self.cmd = [[]]
        self.c_colors = ['red']
        self.c_labels = ['command']
        self.c_xlabel = 'time [s]'
        self.c_ylabel = 'ctrl type [ ]'
        self.c_title = '0: not in cmd / 1: kuam ctrl / 2: px4 ctrl'
        
        self.list_set2 = [self.target, self.cmd]
        self.color_set2 = [self.t_colors, self.c_colors]
        self.label_set2 = [self.t_labels, self.c_labels]
        self.xlabel_set2 = [self.t_xlabel, self.c_xlabel]
        self.ylabel_set2 = [self.t_ylabel, self.c_ylabel]
        self.title_set2 = [self.t_title, self.c_title]
        self.fig2, self.axs2 = plt.subplots(1, 2, num=2)

    '''
    Callback functions
    ''' 
    def ProcessCB(self, timer):
        if self.is_first:
            self.is_first = False
            self.init_time = rospy.Time.now()

        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        if elapse < rospy.Duration(self.elapsed_time):
            pass
        else:
            time_s = rospy.Time.now()
            elapse = time_s - self.init_time

            self.time_s.append(elapse.to_sec())

            global target_id_
            self.target[Val.TARGET.value].append(target_id_)

            global state_machine_cmd_id_
            self.cmd[Val.CMD.value].append(state_machine_cmd_id_)

            if target_id_ == 1 or target_id_ == 2:
                global input_vel_
                global output_vel_
                global target_pos_

                self.x[Val.OUTPUT_POS_X.value].append(target_pos_.x)
                self.y[Val.OUTPUT_POS_Y.value].append(target_pos_.y)
                self.z[Val.OUTPUT_POS_Z.value].append(target_pos_.z)

                self.vx[Val.INPUT_VEL_X.value].append(input_vel_.linear.x)
                self.vx[Val.OUTPUT_VEL_X.value].append(output_vel_.linear.x)
                self.vy[Val.INPUT_VEL_Y.value].append(input_vel_.linear.y)
                self.vy[Val.OUTPUT_VEL_Y.value].append(output_vel_.linear.y)
                self.vz[Val.INPUT_VEL_Z.value].append(input_vel_.linear.z)
                self.vz[Val.OUTPUT_VEL_Z.value].append(output_vel_.linear.z)

            else:
                self.x[Val.OUTPUT_POS_X.value].append(0.0)
                self.y[Val.OUTPUT_POS_Y.value].append(0.0)
                self.z[Val.OUTPUT_POS_Z.value].append(0.0)

                self.vx[Val.INPUT_VEL_X.value].append(0.0)
                self.vx[Val.OUTPUT_VEL_X.value].append(0.0)
                self.vy[Val.INPUT_VEL_Y.value].append(0.0)
                self.vy[Val.OUTPUT_VEL_Y.value].append(0.0)
                self.vz[Val.INPUT_VEL_Z.value].append(0.0)
                self.vz[Val.OUTPUT_VEL_Z.value].append(0.0)

    def SetpointCB(self, msg):
        global input_vel_
        input_vel_ = msg.vel

        global state_machine_cmd_id_
        if not msg.landing_state.is_pass_landing_standby:
            state_machine_cmd_id_ = 0
        else:
            if not msg.landing_state.is_land:
                state_machine_cmd_id_ = 1
            else:
                state_machine_cmd_id_ = 2

    def VelocityCB(self, msg):
        global output_vel_
        output_vel_ = msg.twist

    def TargetMarkerCB(self, msg):
        global target_id_
        if msg.id == 0 or msg.id == 1:
            is_valid = False
            
            msg_pose_stamped = PoseStamped()
            msg_pose_stamped.header = msg.header
            msg_pose_stamped.pose = msg.pose
            # transform from camera_link to base_link
            is_transformed = False
            try:
                transform = tfBuffer_.lookup_transform('base_link', msg_pose_stamped.header.frame_id, rospy.Time())
                is_transformed = True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass

            if is_transformed == True:
                pose_transformed = tf2_geometry_msgs.do_transform_pose(msg_pose_stamped, transform)
                if self.IsValid(pose_transformed):
                    target_pose = pose_transformed.pose
                    is_valid = True
            
            if is_valid:
                target_id_ = msg.id + 1
                global target_pos_
                target_pos_ = target_pose.position
            else:
                target_id_ = 0
        else:
            target_id_ = 0

    def ChatCB(self, msg):
        cmd = msg.msg
        if cmd == 'pause':
            ani.event_source.stop()
            rospy.loginfo("[eval] pause plotting")
        elif cmd == 'play':
            ani2.event_source.stop()
            # ani.event_source.start()
            # ani2.event_source.start()
            rospy.loginfo("[eval] play plotting")
 
    '''
    Plot
    '''
    def PlotInit(self):
        pass

    def UdatePlot(self, frame):
        type = 0
        for i in range(len(self.axs)):
            for j in range(len(self.axs[i])):
                self.axs[i][j].clear()
                ylim = self.GetLimMinMax(self.list_set[type], Y_AXIS_MARGIN[type])
                self.axs[i][j].set_ylim(ylim[0], ylim[1])
                
                for element in range(len(self.list_set[type])):
                    if (len(self.time_s) == len(self.list_set[type][element])):
                        try:
                            self.axs[i][j].plot(self.time_s, self.list_set[type][element], color=self.color_set[type][element], label=self.label_set[type][element])
                        except:
                            pass

                self.axs[i][j].set_xlabel(self.xlabel_set[type])  # Add an x-label to the axes.
                self.axs[i][j].set_ylabel(self.ylabel_set[type])  # Add a y-label to the axes.
                self.axs[i][j].set_title(self.title_set[type])  # Add a title to the axes.            
                self.axs[i][j].legend()  # Add a legend.

                type += 1

    def PlotInit2(self):
        pass

    def UdatePlot2(self, frame):
        type = 0
        for ax in self.axs2:
            ax.clear()
            ylim = self.GetLimMinMax(self.list_set2[type], Y_AXIS_MARGIN2[type])
            ax.set_ylim(ylim[0], ylim[1])
            
            for element in range(len(self.list_set2[type])):
                if (len(self.time_s) == len(self.list_set2[type][element])):
                    try:
                        ax.plot(self.time_s, self.list_set2[type][element], color=self.color_set2[type][element], label=self.label_set2[type][element])
                    except:
                        pass

            ax.set_xlabel(self.xlabel_set2[type])  # Add an x-label to the axes.
            ax.set_ylabel(self.ylabel_set2[type])  # Add a y-label to the axes.
            ax.set_title(self.title_set2[type])  # Add a title to the axes.            
            ax.legend()  # Add a legend.

            type += 1

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

    def GetLimMinMax2(self, lst, margin):
        ylim = [0, BUF_SIZE - 1]
        leng = len(lst)

        # If empty list is exist, return
        if leng == 0:
            return ylim

        list_max = -sys.maxsize - 1
        if max(lst) > list_max:
            list_max = max(lst)

        list_min = sys.maxsize
        if min(lst) < list_min:
            list_min = min(lst)

        ylim = [float(list_min) - margin, float(list_max) + margin]

        return ylim

    def IsValid(self, pose_stamped):
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            z = pose_stamped.pose.position.z

            if (x>1e+100) or (x<-1e+100):
                return False
            if (y>1e+100) or (y<-1e+100):
                return False
            if (z>1e+100) or (z<-1e+100):
                return False
            return True

if __name__ == "__main__":
    if not len(sys.argv) > 1:
        raise Exception ("Please input elapsed time [s].")

    rospy.init_node('comp_input_output_vel_and_relative_pos')
    plotting = Plotting()
    plotting.elapsed_time = int(sys.argv[1])

    tfBuffer_ = tf2_ros.Buffer()
    listener_ = tf2_ros.TransformListener(tfBuffer_)

    target_sub = rospy.Subscriber('/kuam/data/aruco_tracking/target_state', ArucoState, plotting.TargetMarkerCB)
    setpoint_sub = rospy.Subscriber('/kuam/maneuver/state_machine/setpoint', Setpoint, plotting.SetpointCB)
    vel_sub = rospy.Subscriber('/mavros/local_position/velocity_body', TwistStamped, plotting.VelocityCB)
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)
    freq = 30.0
    process_timer = rospy.Timer(rospy.Duration(1.0/freq), plotting.ProcessCB)

    ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
    ani2 = FuncAnimation(plotting.fig2, plotting.UdatePlot2, init_func=plotting.PlotInit2)
    plt.show(block=True) 