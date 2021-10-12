#!/usr/bin/env python3
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from utils import *
from enum import Enum

import tf2_geometry_msgs
import tf2_ros

from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPoint
from kuam_msgs.msg import VehicleState
from uav_msgs.msg import Chat
from mavros_msgs.msg import HomePosition

MARGIN_RATIO = 0.3
BUF_SIZE = 200

class Val(Enum):
    GROUND_TRUTH_X = 0
    ESTIMATED_X = 1
    GROUND_TRUTH_Y = 0
    ESTIMATED_Y = 1
    GROUND_DIST = 0
    ESTIMATED_DIST = 1
    RMSE_X = 0
    RMSE_Y = 0
    RMSE_DIST = 0


is_home_set_ = False
is_detected_ = False
is_start_ = False

home_position_ = GeoPoint()
input_yaw_rate_ = 0.0
output_yaw_rate_ = 0.0
estimated_ = Point()
target_yaw_ = 0.0

class Plotting:
    def __init__(self):
        self.InitPlot()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.x_err_square_list = []
        self.y_err_square_list = []
        self.d_err_square_list = []

    def InitPlot(self):
        self.step_list = []
        self.step = 0
        
        # Home position = target position (lon, lat, ignore z)
        # Convert target position from lon, lat to cartesian coordinate x, y
        # Convert target position from map frame to base_link frame --> ground truth target position

        self.x = [[], []]
        self.x_colors = ['black', 'red']
        self.x_labels = ['ground truth x', 'estimated x']
        self.x_xlabel = 'step []'
        self.x_ylabel = 'distance [m]'
        self.x_title = "Vehicle Position X"

        self.y = [[], []]
        self.y_colors = ['black', 'green']
        self.y_labels = ['ground truth y', 'estimated y']
        self.y_xlabel = 'step []'
        self.y_ylabel = 'distance [m]'
        self.y_title = "Vehicle Position Y"

        self.d = [[], []]
        self.d_colors = ['black', 'blue']
        self.d_labels = ['ground truth dist', 'estimated dist']
        self.d_xlabel = 'step []'
        self.d_ylabel = 'distance [m]'
        self.d_title = "Vehicle Distance"

        self.rx = [[]]
        self.rx_colors = ['black']
        self.rx_labels = ['rmse x']
        self.rx_xlabel = 'step []'
        self.rx_ylabel = 'distance [m]'
        self.rx_title = "RMSE X"

        self.ry = [[]]
        self.ry_colors = ['black']
        self.ry_labels = ['rmse y']
        self.ry_xlabel = 'step []'
        self.ry_ylabel = 'distance [m]'
        self.ry_title = "RMSE Y"

        self.rd = [[]]
        self.rd_colors = ['black']
        self.rd_labels = ['rmse dist']
        self.rd_xlabel = 'step []'
        self.rd_ylabel = 'distance [m]'
        self.rd_title = "RMSE Dist"

        self.list_set = [self.x, self.y, self.d, self.rx, self.ry, self.rd]
        self.color_set = [self.x_colors, self.y_colors, self.d_colors, self.rx_colors, self.ry_colors, self.rd_colors]
        self.label_set = [self.x_labels, self.y_labels, self.d_labels, self.rx_labels, self.ry_labels, self.rd_labels]
        self.xlabel_set = [self.x_xlabel, self.y_xlabel, self.d_xlabel, self.rx_xlabel, self.ry_xlabel, self.rd_xlabel]
        self.ylabel_set = [self.x_ylabel, self.y_ylabel, self.d_ylabel, self.rx_ylabel, self.ry_ylabel, self.rd_ylabel]
        self.title_set = [self.x_title, self.y_title, self.d_title, self.rx_title, self.ry_title, self.rd_title]
        self.fig, self.axs = plt.subplots(2, 3, figsize=(15,9))

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
        global is_detected_
        if not is_start_ or not is_detected_ :
            return

        # Append horizontal
        self.step_list.append(self.step)
        self.step += 1

        # Append 1st raw vertical
        gt = self.GroundTruthPos()
        gt_d = sqrt(gt.x**2 + gt.y**2)

        global estimated_
        estimated_d = sqrt(estimated_.x**2 + estimated_.y**2)

        self.x[Val.GROUND_TRUTH_X.value].append(gt.x)
        self.x[Val.ESTIMATED_X.value].append(estimated_.x)
        self.y[Val.GROUND_TRUTH_Y.value].append(gt.y)
        self.y[Val.ESTIMATED_Y.value].append(estimated_.y)
        self.d[Val.GROUND_DIST.value].append(gt_d)
        self.d[Val.ESTIMATED_DIST.value].append(estimated_d)

        # Append 2nd raw vertical
        self.rx[Val.RMSE_X.value].append(self.RMSE((gt.x-estimated_.x), self.x_err_square_list))
        self.ry[Val.RMSE_Y.value].append(self.RMSE((gt.y-estimated_.y), self.y_err_square_list))
        self.rd[Val.RMSE_DIST.value].append(self.RMSE((gt_d-estimated_d), self.d_err_square_list))

        is_detected_ = False

    def VehicleCB(self, msg):
        # Subscribe vehicle state (check frame id)
        # Convert vehicle position from camera_link to base_link
        # Set estimated target position by using vehicle position of base_link frame id
        global is_detected_
        if not msg.is_detected:
            return

        is_detected_ = True
        try:
            transform = self.tfBuffer.lookup_transform('base_link', 'camera_link', rospy.Time())

            p = PoseStamped()
            p.header.frame_id = 'camera_link'
            p.header.seq = rospy.Time.now()
            p.pose = msg.vehicle
            transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        global estimated_
        estimated_ = transformed_pose.pose.position

    def HomePositionCB(self, msg):
        global is_home_set_
        is_home_set_ = True

        home_position_.latitude = msg.geo.latitude
        home_position_.longitude = msg.geo.longitude
        home_position_.altitude = msg.geo.altitude

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
        data_idx = 0
        line_idx = 0

        for raw_ax in self.axs:
            for col_ax in raw_ax:

                col_ax.set_ylim(self.GetYLim(self.list_set[data_idx]))
                col_ax.set_xlim(self.GetXLim(self.step_list))

                for sub_data_idx in range(len(self.list_set[data_idx])):
                    
                    if len(self.step_list) == len(self.list_set[data_idx][sub_data_idx]):
                        self.line[line_idx].set_data(self.step_list, self.list_set[data_idx][sub_data_idx])
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

    def GetXLim(self, step_list):
        if len(step_list) == 0:
            return [0, 10]
        elif step_list[-1] < 10.0:
            return [0, 10]
        else:
            return [0, step_list[-1]]

    def GroundTruthPos(self):
        try:
            transform = self.tfBuffer.lookup_transform('base_link', 'map', rospy.Time())

            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.seq = rospy.Time.now()
            p.pose.position.x = 0.0
            p.pose.position.y = 0.0
            p.pose.position.z = 0.0
            p.pose.orientation.x = 0.0
            p.pose.orientation.y = 0.0
            p.pose.orientation.z = 0.0
            p.pose.orientation.w = 1.0
            transformed_pose = tf2_geometry_msgs.do_transform_pose(p, transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            pass

        return transformed_pose.pose.position

    def RMSE(self, err, err_square_list):
        err_square_list.append(err**2)

        err_square_sum = 0
        for es in err_square_list:
            err_square_sum += es

        rmse = sqrt(err_square_sum/len(err_square_list))
        return rmse

if __name__ == "__main__":

    rospy.init_node('eval_starex_logging')
    plotting = Plotting()

    # Init subscriber
    home_position_sub = rospy.Subscriber("/mavros/home_position/home", HomePosition, plotting.HomePositionCB)
    rate = rospy.Rate(0.2)
    while not rospy.is_shutdown() and not is_home_set_:
        rospy.logwarn("[eval_starex_logging] Requesting home set")
        rate.sleep()
    rospy.logwarn("[eval_starex_logging] Home set")


    vehicle_sub = rospy.Subscriber("/kuam/data/vehicle_position/vehicle_state", VehicleState, plotting.VehicleCB)
    cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

    freq = 10.0
    process_timer = rospy.Timer(rospy.Duration(1.0/freq), plotting.ProcessCB)

    ani = FuncAnimation(plotting.fig, plotting.UdatePlot)
    plt.show(block=True) 