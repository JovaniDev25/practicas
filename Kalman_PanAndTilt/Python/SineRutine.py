from pyModbusTCP.client import ModbusClient
from pyModbusTCP import utils
import numpy as np

import matplotlib.pyplot as plt
import numpy as np
from itertools import count
from matplotlib.animation import FuncAnimation

mbc = ModbusClient(host="192.168.100.31", port=502, unit_id=1, auto_open=True)
fig, ax = plt.subplots()
index = count()

x_vals = []
y1_vals, y2_vals,y3_vals,y4_vals,y5_vals, = [], [], [], [], []
j = 0
mu, sigma = 0, 1

def concatMB(reg1, reg2):
    E=utils.get_2comp(reg1, 16)
    D=utils.get_2comp(reg2, 16)
    mbStr = str(E)+str('.')+str(abs(D))
    mbFloat = float(mbStr)
    return mbFloat

def getData(regs):
    sx = concatMB(regs[0], regs[1])
    sy = concatMB(regs[2], regs[3])
    sz = concatMB(regs[4], regs[5])
    y1_vals.append(sx)
    y2_vals.append(sy) 
    y3_vals.append(sz+95.89)  

    potVal=utils.get_2comp(regs[8], 16)
    #pVs =(potVal -1230)*0.1
    pVs =(potVal-300)*0.1
    y5_vals.append(pVs) 
    
def sineWave():
    global j
    j = j+1        
    if (j == 360):
        j = 0
    radIn =np.radians(j)
    sinOut = 45 #7.5*np.sin(radIn*10) +80
    y4_vals.append(sinOut)
    return sinOut


def animate(i):
    x_vals.append(next(index))
    regs = mbc.read_holding_registers(1, 9)
    
    if regs:
        getData(regs)
        sinOutInt = int(sineWave())
        mbc.write_multiple_registers(10, [0,60,sinOutInt]) 
           
        max_data_points = 200
        if len(x_vals) > max_data_points:
            x_vals.pop(0)
            #y1_vals.pop(0)
            #y2_vals.pop(0)
            y3_vals.pop(0)
            y4_vals.pop(0)
            y5_vals.pop(0)
        ax.clear()
        ax.plot(x_vals, y4_vals)
        ax.plot(x_vals, y3_vals)
        ax.plot(x_vals, y5_vals)
        ax.set_xlabel('Tiempo [k]')
        ax.set_ylabel('Amplitud ')
        ax.set_title('Comparación de señales')
        ax.legend(['Referencia','Retro IMU',  'Retro Servo']) 
    else:
        print("read error")

ani = FuncAnimation(fig, animate, interval=5)
plt.show()
