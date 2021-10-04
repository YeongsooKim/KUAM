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
from math import pi

from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from kuam_msgs.msg import ArucoState
from kuam_msgs.msg import Setpoint
from uav_msgs.msg import Chat

Y_AXIS_MARGIN = [1.0, 1.0, 3.0]
BUF_SIZE = 200

class Val(Enum):
    ID = 0
    HEIGHT = 0
    YAW = 0

tfBuffer_ = None
listener_ = None
target_id_ = 0
target_height_m_ = 0.0
target_yaw_deg_ = 0.0

class Plotting:
    def __init__(self):
        self.init_time = None
        self.is_first = True

        self.time_s = []
        
        self.target = [[]]
        self.t_colors = ['red']
        self.t_labels = ['target id']
        self.t_xlabel = 'time [s]'
        self.t_ylabel = 'marker id + 1 [ ]'
        self.t_title = 'Target Marker ID'
        
        self.height = [[]]
        self.h_colors = ['green']
        self.h_labels = ['height']
        self.h_xlabel = 'time [s]'
        self.h_ylabel = 'distance [m]'
        self.h_title = "Height from Marker to Drone"

        self.yaw = [[]]
        self.y_colors = ['blue']
        self.y_labels = ['yaw']
        self.y_xlabel = 'time [s]'
        self.y_ylabel = 'yaw [deg]'
        self.y_title = "Marker Yaw"

        self.list_set = [self.target, self.height, self.yaw]
        self.color_set = [self.t_colors, self.h_colors, self.y_colors]
        self.label_set = [self.t_labels, self.h_labels, self.y_labels]
        self.xlabel_set = [self.t_xlabel, self.h_xlabel, self.y_xlabel]
        self.ylabel_set = [self.t_ylabel, self.h_ylabel, self.y_ylabel]
        self.title_set = [self.t_title, self.h_title, self.y_title]
        self.fig, self.axs = plt.subplots(1, 3)

    '''
    Callback functions
    ''' 
    def ProcessCB(self, timer):
        if self.is_first:
            self.is_first = False
            self.init_time = rospy.Time.now()

        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        self.time_s.append(elapse.to_sec())

        global target_id_
        self.target[Val.ID.value].append(target_id_)

        global target_height_m_
        global target_yaw_deg_
        if not target_id_ == -1:
            self.height[Val.HEIGHT.value].append(target_height_m_)
            self.yaw[Val.YAW.value].append(target_yaw_deg_)
        else:
            self.height[Val.HEIGHT.value].append(0.0)
            self.yaw[Val.YAW.value].append(0.0)

    def TargetMarkerCB(self, msg):
        global target_id_
        if msg.is_detected:
            # transform from camera_link to base_link
            try:
                transform = tfBuffer_.lookup_transform('base_link', msg.header.frame_id, rospy.Time())

                p = PoseStamped()
                p.header = msg.header
                p.pose = msg.pose
                transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)
                
                if self.IsValid(transformed_pose):
                    target_id_ = msg.id

                    global target_height_m_
                    target_height_m_ = transformed_pose.pose.position.z

                    global target_yaw_deg_
                    q = transformed_pose.pose.orientation
                    target_yaw_deg_ = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]*180.0/pi + pi
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                pass
        else:
            target_id_ = -1

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
    def PlotInit(self):
        pass
        # for ax in self.axs:
        #     ax.set_xlim(0, BUF_SIZE - 1)
        #     ax.set_ylim(0.0, 1.0)

    def UdatePlot(self, frame):
        for type, ax in enumerate(self.axs):
            ax.clear()
            ylim = self.GetLimMinMax(self.list_set[type], Y_AXIS_MARGIN[type])
            ax.set_ylim(ylim[0], ylim[1])
            
            for element in range(len(self.list_set[type])):
                if (len(self.time_s) == len(self.list_set[type][element])):
                    try:
                        ax.plot(self.time_s, self.list_set[type][element], color=self.color_set[type][element], label=self.label_set[type][element])
                    except:
                        pass

            ax.set_xlabel(self.xlabel_set[type])  # Add an x-label to the axes.
            ax.set_ylabel(self.ylabel_set[type])  # Add a y-label to the axes.
            ax.set_title(self.title_set[type])  # Add a title to the axes.            
            ax.legend()  # Add a legend.

    '''
    Util functions
    '''
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

if __name__ == "__main__":
    rospy.init_node('eval_orientation')
    plotting = Plotting()

    tfBuffer_ = tf2_ros.Buffer()
    listener_ = tf2_ros.TransformListener(tfBuffer_)

    target_sub = rospy.Subscriber('/kuam/data/aruco_tracking/target_state', ArucoState, plotting.TargetMarkerCB)
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)
    freq = 30.0
    process_timer = rospy.Timer(rospy.Duration(1.0/freq), plotting.ProcessCB)

    ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
    plt.show(block=True) 