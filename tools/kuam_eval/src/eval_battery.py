#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from math import sqrt
import sys
from enum import Enum

from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import PoseStamped
from uav_msgs.msg import Chat

Y_AXIS_MARGIN = 0.3
BUF_SIZE = 200

class State(Enum):
    PERCENTAGE = 0
    VOLTAGE = 1
    CURRENT = 2


class Plotting:
    def __init__(self):
        self.init_time = None
        self.is_first = True

        self.time_s = []
        self.state = [[]]
        self.colors = ['red']
        self.labels = ['percentage']

        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('time [s]')  # Add an x-label to the axes.
        self.ax.set_ylabel('percentage [%]')  # Add a y-label to the axes.
        self.ax.set_title("Battery Remaining Percentage")  # Add a title to the axes.


    def PlotInit(self):
        self.ax.set_xlim(0, BUF_SIZE - 1)
        self.ax.set_ylim(0.0 - Y_AXIS_MARGIN, 1.0 + Y_AXIS_MARGIN)


    def BatteryCB(self, msg):
        if self.is_first:
            self.is_first = False
            self.init_time = rospy.Time.now()

        time_s = rospy.Time.now()
        elapse = time_s - self.init_time

        self.time_s.append(elapse.to_sec())
        self.state[State.PERCENTAGE.value].append(msg.percentage)
        

    def ChatCB(self, msg):
        cmd = msg.msg
        if cmd == 'pause':
            ani.event_source.stop()
            rospy.loginfo("[eval] pause plotting")
        elif cmd == 'play':
            ani.event_source.start()
            rospy.loginfo("[eval] play plotting")


    def UdatePlot(self, frame):
        self.ax.clear()
        self.ax.set_xlabel('time [s]')  # Add an x-label to the axes.
        self.ax.set_ylabel('percentage [%]')  # Add a y-label to the axes.
        self.ax.set_title("Battery Remaining Percentage")  # Add a title to the axes.
        self.ax.set_ylim(0.0 - Y_AXIS_MARGIN, 1.0 + Y_AXIS_MARGIN)

        for i in range(len(self.state)):
            if (len(self.time_s) == len(self.state[i])):
                self.ax.plot(self.time_s, self.state[i], color=self.colors[i], label=self.labels[i])
            
        self.ax.legend()  # Add a legend.


rospy.init_node('eval_battery')
plotting = Plotting()

vel_sub = rospy.Subscriber('/mavros/battery', BatteryState, plotting.BatteryCB)
cmd_sub = rospy.Subscriber("/kuam/data/chat/command", Chat, plotting.ChatCB)

ani = FuncAnimation(plotting.fig, plotting.UdatePlot, init_func=plotting.PlotInit)
plt.show(block=True) 