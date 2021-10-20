#!/usr/bin/env python3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
import sys
from utils import *
from enum import Enum
import numpy as np

import tf2_geometry_msgs
import tf2_ros

# Message
from uav_msgs.msg import Chat
from kuam_msgs.msg import ArucoState
from kuam_msgs.msg import ArucoStates
from geometry_msgs.msg import Pose


output = np.zeros((10), dtype=np.complex128)

a = np.complex(0, 0)
print(abs(a))