#!/usr/bin/env python

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import numpy as np

#style.use("fivethirtyeigth")

fig = plt.figure()
ax1 = fig.add_subplot(1,1,1)

def animate(i):
    grap_data = open("../pid.log",'r').read()
    lines = grap_data.split('\n')
    lines.pop(0)
    xs = []
    ys = []
    sts = []
    for line in lines:
        try:
            t, cte, steer, th, kp,kd,ki, pSat, ctrlout, satOut, saturated = map(lambda x: float(x.split(' ')[1]),  line.strip().split(","))
            ys.append(cte)
            xs.append(t)
            sts.append(steer)
            if len(xs) > 700:
                xs.pop(0)
                ys.pop(0)
                sts.pop(0)
        except:
            pass
    ax1.clear()
    ax1.plot(xs, ys)
    ax1.plot(xs, sts)



ani = animation.FuncAnimation(fig, animate, interval=100)
plt.axis('on')
plt.show()
