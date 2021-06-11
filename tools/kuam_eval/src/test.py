#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt

fig, axs = plt.subplots(1, 2)
axs[0].set_xlabel('time [s]')  # Add an x-label to the axes.
axs[0].set_ylabel('height [m]')  # Add a y-label to the axes.
axs[0].set_title("Vehicle to Marker Height")  # Add a title to the axes.

axs[1].set_xlabel('time [s]')  # Add an x-label to the axes.
axs[1].set_ylabel('velocity [m/s]')  # Add a y-label to the axes.
axs[1].set_title("Vehicle Linear Z Vel")  # Add a title to the axes.

freq = 30.0
dt = 1.0/freq
last_time = 100
init_z = 5.0

def Z(t):
    a = init_z/last_time**4
    z = a*(t-last_time)**4
    return z

times = []
z = []
cnt = 1
while cnt*dt < last_time:
    times.append(cnt*dt)
    z.append(Z(cnt*dt))
    cnt += 1

cnt = 1
z_vel = []
while cnt*dt < last_time:
    if cnt + 1 < len(z):
        vel = (z[cnt+1] - z[cnt])/dt
        z_vel.append(vel)
    else:
        vel = z[cnt-1]
        z_vel.append(vel)
    cnt += 1


axs[0].plot(times, z)
axs[1].plot(times, z_vel)
plt.show(block=True) 