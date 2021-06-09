#!/usr/bin/env python
# license removed for brevity
import matplotlib.pyplot as plt

fig, ax = plt.subplots()
ax.set_xlabel('time [s]')  # Add an x-label to the axes.
ax.set_ylabel('velocity [m/s]')  # Add a y-label to the axes.
ax.set_title("Ego Body Velocity")  # Add a title to the axes.

freq = 30.0
dt = 1.0/freq
last_time = 100

def Z(t):
    a = 5.0/100**4
    z = a*(t-100)**4
    return z

times = []
z = []
cnt = 1
while cnt*dt < 100:
    times.append(cnt*dt)
    z.append(Z(cnt*dt))
    cnt += 1


ax.plot(times, z)
plt.show(block=True) 