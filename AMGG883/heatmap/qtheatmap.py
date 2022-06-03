import serial
import time
import threading
import readchar
import json
import tkinter as tk
import numpy as np
import sys
import seaborn as sb
import matplotlib as mpl
import matplotlib.pyplot as plt
from PyQt5 import QtWidgets, QtCore
import pyqtgraph as pg
from colour import Color
from pgcolorbar.colorlegend import ColorLegendItem
import cv2

HEIGHT = 240
WIDTH = 240
COLOUR_DEPTH = 1024
MIN_TEMP = 26.0
MAX_TEMP = 32.0


row1 = [0, 0, 0, 0, 0, 0, 0, 0]
row2 = [0, 0, 0, 0, 0, 0, 0, 0]
row3 = [0, 0, 0, 0, 0, 0, 0, 0]
row4 = [0, 0, 0, 0, 0, 0, 0, 0]
row5 = [0, 0, 0, 0, 0, 0, 0, 0]
row6 = [0, 0, 0, 0, 0, 0, 0, 0]
row7 = [0, 0, 0, 0, 0, 0, 0, 0]
row8 = [0, 0, 0, 0, 0, 0, 0, 0]

data = np.zeros((8, 8))

s = serial.Serial("/dev/ttyACM0", 115200)

app = QtWidgets.QApplication(sys.argv)

def map_value(x, inMin, inMax, outMin, outMax):
    return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin

def heat_map():
    data[0] = row8
    data[1] = row7
    data[2] = row6
    data[3] = row5
    data[4] = row4
    data[5] = row3
    data[6] = row2
    data[7] = row1

    newData = np.zeros((24, 24))
    newData = cv2.resize(data[0], dsize=(24, 24), interpolation=cv2.INTER_LANCZOS4)
    print(newData)


    window.show()
    image.setImage(data)

    #print(data)
    print("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")


def read_loop():
    data = s.readline()
    # print(data)
    data = data.decode()
    
    # print(str(data))
    line = data.split(',')
    # print(line)
    line = line[:-1]
    index = line[0]
    del line[0]
    
    for i in range(8):
        if index == '0':
            row1[i] = float(line[i])
        elif index == '1':
            row2[i] = float(line[i])
        elif index == '2':
            row3[i] = float(line[i])
        elif index == '3':
            row4[i] = float(line[i])
        elif index == '4':
            row5[i] = float(line[i])
        elif index == '5':
            row6[i] = float(line[i])
        elif index == '6':
            row7[i] = float(line[i])
        elif index == '7':
            row8[i] = float(line[i])
    
    heat_map()

    
    


timer = QtCore.QTimer()
timer.setInterval(5)
timer.timeout.connect(read_loop)
timer.start()



window = pg.GraphicsLayoutWidget()
blue, red = Color('blue'), Color('Red')
colors = blue.range_to(red, COLOUR_DEPTH)
colors_array = np.array([np.array(color.get_rgb())*255 for color in colors])
look_up_table = colors_array.astype(np.uint8)


image = pg.ImageItem()
image.setOpts(axisOrder='row-major')
image.setLookupTable(look_up_table)
image.setImage(data)

view_box = pg.ViewBox()
view_box.setAspectLocked(lock=True)
view_box.addItem(image)

plot = pg.PlotItem(viewBox=view_box)
window.addItem(plot)
sys.exit(app.exec_())