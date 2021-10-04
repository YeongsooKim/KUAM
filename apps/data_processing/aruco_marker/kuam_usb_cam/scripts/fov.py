#!/usr/bin/env python
# license removed for brevity
import numpy as np
import matplotlib.pyplot as plt
from math import tan
from math import pi

W_THETA_DEG = 37.0
H_THETA_DEG = 29.0
DEG2RAD = pi/180.0

MAX_H_M = 2.0

alt_list = np.linspace(MAX_H_M, 0, 100)

# print(alt_list)
width_list = []
for alt in alt_list:
    w = alt*tan(W_THETA_DEG*DEG2RAD)
    width_list.append(w)

height_list = []
for alt in alt_list:
    h = alt*tan(H_THETA_DEG*DEG2RAD)
    height_list.append(h)


fig, axs = plt.subplots(1, 2)

axs[0].plot(alt_list, width_list)
axs[0].set_xlim(max(alt_list), min(alt_list))
axs[0].set_ylabel("distance [m]")  # Add an x-label to the axes.
axs[0].set_xlabel("altitude [m]")  # Add a y-label to the axes.
axs[0].set_title("Horizontal Angle: 37 deg")

axs[1].plot(alt_list, height_list)
axs[1].set_xlim(max(alt_list), min(alt_list))
axs[1].set_ylabel("distance [m]")
axs[1].set_xlabel("altitude [m]")  # Add a y-label to the axes.
axs[1].set_title("Veritcal Angle: 29 deg")

# display the graph
fig.tight_layout()
plt.show()