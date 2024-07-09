from pyModbusTCP.client import ModbusClient
from pyModbusTCP import utils

import matplotlib.pyplot as plt
import numpy as np
from itertools import count
from matplotlib.animation import FuncAnimation

mbc = ModbusClient(host="192.168.100.30", port=502, unit_id=1, auto_open=True)
#mbc = ModbusClient(host="192.168.43.90", port=502, unit_id=1, auto_open=True)

fig, axs = plt.subplots(2, 2)
x_vals= []
y_vals= []
yd_vals= []
yml_vals= []
ymr_vals= []
aPosD= []
aPosC = []
aVelD= []
aVelC = []
index = count()


def animate(i):
    x_vals.append(next(index))
    regs = mbc.read_holding_registers(1, 6)

    if regs:
        aPosD=utils.get_2comp(regs[0], 16)
        aPosC=utils.get_2comp(regs[1], 16)
        s = str(aPosD)+str('.')+str(abs(aPosC))
        y_vals.append(float(s))   
        aVelD=utils.get_2comp(regs[2], 16)
        aVelC=utils.get_2comp(regs[3], 16)
        v = str(aVelD)+str('.')+str(abs(aVelC))
        yd_vals.append(float(v))  
        yml_vals.append(utils.get_2comp(regs[4], 16))      
        ymr_vals.append(utils.get_2comp(regs[5], 16)) 
        max_data_points = 2000
        if len(x_vals) > max_data_points:
            x_vals.pop(0)
            y_vals.pop(0)
        axs[0, 0].clear()   
        axs[0, 1].clear()   
        axs[1, 0].clear()   
        axs[1, 1].clear()      
        axs[0, 0].plot(x_vals, y_vals)
        axs[0, 0].set_title('Angular Position')
        axs[0, 1].plot(x_vals, yd_vals, 'tab:orange')
        axs[0, 1].set_title('Angular Velocity')
        axs[1, 0].plot(x_vals, ymr_vals, 'tab:green')
        axs[1, 0].set_title('Motor R PWM')
        axs[1, 1].plot(x_vals, yml_vals, 'tab:red')
        axs[1, 1].set_title('Motor L PWM')

    else:
        print("read error")
ani = FuncAnimation(fig, animate, interval=1)  # Update every 1000 ms (1 second)
plt.show()
